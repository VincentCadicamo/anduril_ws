"""
Camera bridge: receives the simulator's chunked JPEG video stream over UDP
and republishes complete frames as sensor_msgs/Image on /cam0/image_raw.

Each UDP packet is a 24-byte little-endian header followed
by a slice of a JPEG. A complete frame is reassembled by collecting all
chunks sharing the same frame_id.

Header layout (24 bytes, little-endian):
  frame_id        uint32  unique sequence id for the frame
  chunk_id        uint16  index of this packet within the frame
  total_chunks    uint16  total packets required to assemble the frame
  jpeg_size       uint32  size of the complete reconstructed JPEG
  payload_size    uint32  size of the JPEG slice in this packet
  sim_time_ns     uint64  simulator capture timestamp, nanoseconds
"""

import socket
import struct
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


HEADER_FORMAT = "<IHHIIQ"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
assert HEADER_SIZE == 24

# The sim's frames at 640 x 360 JPEG are well under these.
MAX_JPEG_BYTES = 4 * 1024 * 1024     # 4 MiB ceiling per frame
MAX_CHUNKS_PER_FRAME = 256           # any frame fragmenting more than this is malformed
RECV_BUFFER_BYTES = 65536            # one UDP datagram, max


@dataclass
class FrameAssembly:
    """In-progress reassembly of one JPEG frame keyed by frame_id."""
    total_chunks: int
    jpeg_size: int
    sim_time_ns: int
    chunks: dict = field(default_factory=dict)   # chunk_id -> bytes

    def complete(self) -> bool:
        return len(self.chunks) == self.total_chunks

    def assemble(self) -> bytes:
        return b"".join(self.chunks[i] for i in range(self.total_chunks))


class CameraBridge(Node):
    def __init__(self):
        super().__init__("camera_bridge")

        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 5600)
        self.declare_parameter("image_topic", "/cam0/image_raw")
        self.declare_parameter("frame_id", "cam0")

        host = self.get_parameter("listen_host").value
        port = self.get_parameter("listen_port").value
        topic = self.get_parameter("image_topic").value
        self.camera_frame_id = self.get_parameter("frame_id").value

        # UDP socket. Non-blocking with a small timeout so we can yield
        # back to rclpy's executor periodically.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.settimeout(0.01)
        self.sock.bind((host, port))
        self.get_logger().info(f"Listening for vision UDP on {host}:{port}")

        self.publisher = self.create_publisher(Image, topic, 10)

        # In-progress reassemblies. Cleared whenever a new frame_id arrives,
        # so partial frames are dropped rather than stitched across boundaries.
        self.assemblies: dict[int, FrameAssembly] = {}
        self.latest_frame_id: int | None = None

        # Drive the receive loop from a timer so rclpy stays in control of
        # the event loop. 1 ms timer and non-blocking recv means no busy wait.
        self.create_timer(0.001, self._tick)

        # Stats logged occasionally to make problems visible.
        self.frames_published = 0
        self.frames_dropped_incomplete = 0
        self.frames_dropped_malformed = 0
        self.create_timer(5.0, self._log_stats)

    def _tick(self):
        # Drain whatever packets are currently sitting in the socket buffer.
        # Bounded loop prevents us from starving rclpy if the sim ever bursts.
        for _ in range(64):
            try:
                packet, _addr = self.sock.recvfrom(RECV_BUFFER_BYTES)
            except socket.timeout:
                return
            except BlockingIOError:
                return
            self._handle_packet(packet)

    def _handle_packet(self, packet: bytes):
        if len(packet) < HEADER_SIZE:
            self.frames_dropped_malformed += 1
            return

        try:
            frame_id, chunk_id, total_chunks, jpeg_size, payload_size, sim_time_ns = \
                struct.unpack(HEADER_FORMAT, packet[:HEADER_SIZE])
        except struct.error:
            self.frames_dropped_malformed += 1
            return

        payload = packet[HEADER_SIZE:HEADER_SIZE + payload_size]
        if len(payload) != payload_size:
            self.frames_dropped_malformed += 1
            return

        # Bounds checks to protect against malformed streams.
        if (total_chunks == 0
                or total_chunks > MAX_CHUNKS_PER_FRAME
                or jpeg_size == 0
                or jpeg_size > MAX_JPEG_BYTES
                or chunk_id >= total_chunks):
            self.frames_dropped_malformed += 1
            return

        # A new frame arrived before the previous one finished: drop the old.
        # This is the right policy for low-latency video, where stale frames
        # are worse than missing frames.
        if self.latest_frame_id is not None and frame_id != self.latest_frame_id:
            stale = self.assemblies.pop(self.latest_frame_id, None)
            if stale is not None and not stale.complete():
                self.frames_dropped_incomplete += 1
        self.latest_frame_id = frame_id

        asm = self.assemblies.get(frame_id)
        if asm is None:
            asm = FrameAssembly(
                total_chunks=total_chunks,
                jpeg_size=jpeg_size,
                sim_time_ns=sim_time_ns,
            )
            self.assemblies[frame_id] = asm

        asm.chunks[chunk_id] = payload

        if asm.complete():
            jpeg = asm.assemble()
            self.assemblies.pop(frame_id, None)
            self._publish_frame(jpeg, asm.sim_time_ns)

    def _publish_frame(self, jpeg_bytes: bytes, sim_time_ns: int):
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            self.frames_dropped_malformed += 1
            return

        h, w = img.shape[:2]
        msg = Image()
        msg.header.stamp.sec = sim_time_ns // 1_000_000_000
        msg.header.stamp.nanosec = sim_time_ns % 1_000_000_000
        msg.header.frame_id = self.camera_frame_id
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = img.tobytes()

        self.publisher.publish(msg)
        self.frames_published += 1

    def _log_stats(self):
        self.get_logger().info(
            f"frames published={self.frames_published} "
            f"dropped_incomplete={self.frames_dropped_incomplete} "
            f"dropped_malformed={self.frames_dropped_malformed} "
            f"in-flight_assemblies={len(self.assemblies)}"
        )


def main():
    rclpy.init()
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()