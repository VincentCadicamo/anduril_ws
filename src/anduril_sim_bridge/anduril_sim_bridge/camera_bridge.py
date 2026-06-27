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

Reassembly keeps a small window of concurrently in-flight frames rather than
a single one. UDP can reorder packets across frame boundaries, so a strict
"new frame_id means drop the previous" policy throws away frames that were
only one out-of-order packet from completing. Instead we hold up to
MAX_INFLIGHT_FRAMES assemblies and evict the oldest only when that bound is
exceeded.
"""

import socket
import struct
from collections import OrderedDict
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

# How many partially-assembled frames to keep alive at once. Sized to cover
# realistic UDP reordering depth across frame boundaries without unbounded
# memory growth. Raised to 128 because under flight load the previous cap of 32
# was pinned full with frames evicted before completing, starving OpenVINS of
# consecutive frames for disparity. Watch evicted_avg_complete in the stats: if
# it stays near 1.0 the window may still be too small; if it drops low, the
# bottleneck is genuine packet loss instead, not window size.
MAX_INFLIGHT_FRAMES = 128


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
        self.declare_parameter("max_inflight_frames", MAX_INFLIGHT_FRAMES)

        host = self.get_parameter("listen_host").value
        port = self.get_parameter("listen_port").value
        topic = self.get_parameter("image_topic").value
        self.camera_frame_id = self.get_parameter("frame_id").value
        self.max_inflight = int(self.get_parameter("max_inflight_frames").value)

        # UDP socket. Non-blocking with a small timeout so we can yield
        # back to rclpy's executor periodically.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Large receive buffer to absorb bursts under flight load. The kernel
        # may cap this below the request (see net.core.rmem_max); the
        # diagnostic below logs what was actually granted so we can tell if
        # the OS clamped it.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        actual_rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.sock.settimeout(0.01)
        self.sock.bind((host, port))
        self.get_logger().info(f"Listening for vision UDP on {host}:{port}")
        # Report the actually-granted receive buffer. If this is far below the
        # requested 8 MiB, the kernel clamped it (raise net.core.rmem_max), and
        # buffer overflow may be a source of incomplete frames under load.
        self.get_logger().info(
            f"Socket SO_RCVBUF granted: {actual_rcvbuf} bytes "
            f"({actual_rcvbuf / (1 << 20):.1f} MiB)"
        )

        self.publisher = self.create_publisher(Image, topic, 10)

        # In-progress reassemblies, ordered by insertion (oldest first) so we
        # can evict the oldest when the in-flight bound is exceeded.
        self.assemblies: "OrderedDict[int, FrameAssembly]" = OrderedDict()
        # Frames at or below this id have been evicted off the bottom of the
        # window and will never reopen; their late chunks are dropped. Starts
        # below any real frame_id so the first frames are never pre-retired.
        self.retire_floor: int = -1

        # Drive the receive loop from a timer so rclpy stays in control of
        # the event loop. 1 ms timer and non-blocking recv means no busy wait.
        self.create_timer(0.001, self._tick)

        # Stats logged occasionally to make problems visible.
        self.frames_published = 0
        self.frames_dropped_incomplete = 0
        self.frames_dropped_malformed = 0
        self.frames_dropped_straggler = 0
        # DIAGNOSTIC accumulators: how complete evicted-incomplete frames were.
        self._evict_have = 0    # chunks present across evicted frames
        self._evict_need = 0    # chunks required across evicted frames
        self._evict_count = 0   # number of incomplete evictions
        self.create_timer(5.0, self._log_stats)

    def _tick(self):
        # Drain whatever packets are currently sitting in the socket buffer.
        # Bounded loop prevents us from starving rclpy if the sim ever bursts.
        # Raised from 64 to 512: under flight load the sim emits many chunks
        # per millisecond, and a low drain limit left packets queued in the
        # kernel buffer until it overflowed, dropping chunks (which showed up
        # as incomplete frames). 512 keeps the receiver ahead without starving
        # the executor, since recvfrom returns immediately once the buffer
        # empties (socket.timeout / BlockingIOError breaks the loop).
        for _ in range(512):
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

        # Drop chunks only for frames that have fallen off the bottom of the
        # window (genuinely retired). A frame_id below the floor is one we
        # already published or evicted and will never reopen. We do NOT treat
        # "lower than the newest completed frame" as retired: under reordering,
        # lower-id frames are often still legitimately assembling.
        if frame_id <= self.retire_floor:
            self.frames_dropped_straggler += 1
            return

        asm = self.assemblies.get(frame_id)
        if asm is None:
            asm = FrameAssembly(
                total_chunks=total_chunks,
                jpeg_size=jpeg_size,
                sim_time_ns=sim_time_ns,
            )
            self.assemblies[frame_id] = asm
            # Keep the window bounded. Evict oldest frames beyond the limit;
            # those are ones whose chunks stopped arriving (genuine loss).
            self._enforce_inflight_bound()

        asm.chunks[chunk_id] = payload

        if asm.complete():
            jpeg = asm.assemble()
            del self.assemblies[frame_id]
            # A completed frame is published but does NOT raise the retire
            # floor for lower ids — those may still be assembling. The floor
            # only advances on eviction (see _enforce_inflight_bound).
            self._publish_frame(jpeg, asm.sim_time_ns)

    def _enforce_inflight_bound(self):
        # OrderedDict preserves insertion order, but packets can arrive out of
        # frame_id order, so insertion order isn't strictly id order. Evict the
        # lowest frame_id (true oldest) when over the bound, and raise the
        # retire floor to it so its late chunks are dropped rather than
        # reopening the frame.
        while len(self.assemblies) > self.max_inflight:
            oldest_id = min(self.assemblies.keys())
            old_asm = self.assemblies.pop(oldest_id)
            if not old_asm.complete():
                self.frames_dropped_incomplete += 1
                # DIAGNOSTIC: record how complete the evicted frame was. If
                # evicted frames are nearly complete (e.g. 90%+), the window is
                # too small / evicting too early. If they're sparse (e.g. <50%),
                # chunks are genuinely not arriving (UDP loss). Sampled to avoid
                # log spam.
                self._evict_have += len(old_asm.chunks)
                self._evict_need += old_asm.total_chunks
                self._evict_count += 1
            if oldest_id > self.retire_floor:
                self.retire_floor = oldest_id

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
        # Average completeness of evicted-incomplete frames. ~1.0 means frames
        # are evicted nearly complete (window too small) -> raise max_inflight.
        # Low (e.g. 0.5) means chunks genuinely missing (UDP loss) -> the cap
        # won't help; need a bigger socket buffer or lower send rate.
        if self._evict_count > 0:
            avg_complete = self._evict_have / max(1, self._evict_need)
            evict_info = (f" | evicted_avg_complete={avg_complete:.2f} "
                          f"over {self._evict_count} frames")
        else:
            evict_info = ""
        self.get_logger().info(
            f"frames published={self.frames_published} "
            f"dropped_incomplete={self.frames_dropped_incomplete} "
            f"dropped_malformed={self.frames_dropped_malformed} "
            f"dropped_straggler={self.frames_dropped_straggler} "
            f"in-flight_assemblies={len(self.assemblies)}{evict_info}"
        )
        # Reset the eviction sample window each report so the ratio reflects
        # recent behavior, not lifetime average.
        self._evict_have = 0
        self._evict_need = 0
        self._evict_count = 0


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