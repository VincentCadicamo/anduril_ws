import socket
import struct
from collections import OrderedDict
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


HEADER_FORMAT = "<IHHIIQ"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
assert HEADER_SIZE == 24

MAX_JPEG_BYTES = 4 * 1024 * 1024
MAX_CHUNKS_PER_FRAME = 256
RECV_BUFFER_BYTES = 65536

MAX_INFLIGHT_FRAMES = 128


@dataclass
class FrameAssembly:
    """In-progress reassembly of one JPEG frame keyed by frame_id."""
    total_chunks: int
    jpeg_size: int
    sim_time_ns: int
    chunks: dict = field(default_factory=dict)

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
        self.declare_parameter("max_fps", 30.0)

        host = self.get_parameter("listen_host").value
        port = self.get_parameter("listen_port").value
        topic = self.get_parameter("image_topic").value
        self.camera_frame_id = self.get_parameter("frame_id").value
        self.max_inflight = int(self.get_parameter("max_inflight_frames").value)
        max_fps = self.get_parameter("max_fps").value
        self._min_frame_interval_ns = int(1e9 / max_fps) if max_fps > 0 else 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        actual_rcvbuf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.sock.setblocking(False)
        self.sock.bind((host, port))
        self.get_logger().info(f"Listening for vision UDP on {host}:{port}")
        self.get_logger().info(
            f"Socket SO_RCVBUF granted: {actual_rcvbuf} bytes "
            f"({actual_rcvbuf / (1 << 20):.1f} MiB)"
        )

        self.publisher = self.create_publisher(Image, topic, _IMAGE_QOS)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, topic.replace("image_raw", "camera_info"), _IMAGE_QOS
        )
        self._camera_info = self._build_camera_info()

        self.assemblies: "OrderedDict[int, FrameAssembly]" = OrderedDict()
        self.retire_floor: int = -1
        self._sim_clock_offset_ns: int | None = None
        self._last_published_sim_ns: int = 0
        self.frames_dropped_rate_limit = 0

        self.create_timer(0.001, self._tick)

        self.frames_published = 0
        self.frames_dropped_incomplete = 0
        self.frames_dropped_malformed = 0
        self.frames_dropped_straggler = 0
        self._evict_have = 0
        self._evict_need = 0
        self._evict_count = 0
        self.create_timer(5.0, self._log_stats)

    def _tick(self):
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

        if (total_chunks == 0
                or total_chunks > MAX_CHUNKS_PER_FRAME
                or jpeg_size == 0
                or jpeg_size > MAX_JPEG_BYTES
                or chunk_id >= total_chunks):
            self.frames_dropped_malformed += 1
            return

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
            self._enforce_inflight_bound()

        asm.chunks[chunk_id] = payload

        if asm.complete():
            del self.assemblies[frame_id]
            elapsed = asm.sim_time_ns - self._last_published_sim_ns
            if self._min_frame_interval_ns and elapsed < self._min_frame_interval_ns:
                self.frames_dropped_rate_limit += 1
                return
            self._publish_frame(asm.assemble(), asm.sim_time_ns)

    def _enforce_inflight_bound(self):
        while len(self.assemblies) > self.max_inflight:
            oldest_id = next(iter(self.assemblies))
            old_asm = self.assemblies.pop(oldest_id)
            if not old_asm.complete():
                self.frames_dropped_incomplete += 1
                self._evict_have += len(old_asm.chunks)
                self._evict_need += old_asm.total_chunks
                self._evict_count += 1
            if oldest_id > self.retire_floor:
                self.retire_floor = oldest_id

    def _build_camera_info(self) -> CameraInfo:
        # Intrinsics from kalibr_imucam_chain.yaml: pinhole, no distortion, 640x360
        ci = CameraInfo()
        ci.header.frame_id = self.camera_frame_id
        ci.width = 640
        ci.height = 360
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # K: [fx,  0, cx,  0, fy, cy,  0,  0,  1]
        ci.k = [320.0, 0.0, 320.0, 0.0, 320.0, 180.0, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # P: [fx,  0, cx, 0,   0, fy, cy, 0,   0,  0,  1, 0]
        ci.p = [320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 180.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _publish_frame(self, jpeg_bytes: bytes, sim_time_ns: int):
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.frames_dropped_malformed += 1
            return

        # Lock the sim-to-ROS clock offset on the first frame so camera timestamps
        # align with IMU timestamps (which the TimeAligner maps to ROS wall time).
        # WSL2 clock can drift several seconds from the sim's Windows clock.
        if self._sim_clock_offset_ns is None:
            ros_now_ns = self.get_clock().now().nanoseconds
            self._sim_clock_offset_ns = ros_now_ns - sim_time_ns
            self.get_logger().info(
                f"[DIAG] Sim-to-ROS clock offset locked: "
                f"{self._sim_clock_offset_ns / 1e9:+.3f}s"
            )

        stamp_ns = sim_time_ns + self._sim_clock_offset_ns

        h, w = img.shape[:2]
        msg = Image()
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = self.camera_frame_id
        msg.height = h
        msg.width = w
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = w
        msg.data = img.tobytes()

        self._last_published_sim_ns = sim_time_ns
        self.publisher.publish(msg)

        ci = self._camera_info
        ci.header.stamp = msg.header.stamp
        self.camera_info_pub.publish(ci)

        self.frames_published += 1
        if self.frames_published == 1:
            self.get_logger().info(
                f"[DIAG] First frame published: sim_time_ns={sim_time_ns} "
                f"corrected_stamp_ns={stamp_ns}"
            )

    def _log_stats(self):
        if self._evict_count > 0:
            avg_complete = self._evict_have / max(1, self._evict_need)
            evict_info = (f" | evicted_avg_complete={avg_complete:.2f} "
                          f"over {self._evict_count} frames")
        else:
            evict_info = ""
        self.get_logger().info(
            f"frames published={self.frames_published} "
            f"dropped_rate_limit={self.frames_dropped_rate_limit} "
            f"dropped_incomplete={self.frames_dropped_incomplete} "
            f"dropped_malformed={self.frames_dropped_malformed} "
            f"dropped_straggler={self.frames_dropped_straggler} "
            f"in-flight_assemblies={len(self.assemblies)}{evict_info}"
        )
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