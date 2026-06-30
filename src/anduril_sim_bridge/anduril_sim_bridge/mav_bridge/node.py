"""
node.py: the ROS node. Thin. It wires publishers/subscribers to the transport,
converters, time aligner, and controller - and does nothing else.

All the testable logic lives in the other modules. This file is plumbing:
declare params, build the pieces, register handlers, run timers.
"""

from __future__ import annotations

import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
)

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import AttitudeTarget
from anduril_msgs.msg import RaceStatus, TrackData, Collision

from .transport import MavlinkTransport
from .control import OffboardController
from .time_align import TimeAligner
from . import converters


class MavBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mav_bridge")

        # ----- parameters -----
        self.declare_parameter("connection_string", "udpin:0.0.0.0:14550")
        self.declare_parameter("imu_topic", "/imu0")
        self.declare_parameter("imu_frame_id", "imu0")
        self.declare_parameter("heartbeat_period_s", 1.0)
        self.declare_parameter("setpoint_topic", "/setpoint/attitude_target")
        self.declare_parameter("auto_offboard_arm", True)
        self.declare_parameter("odom_frame_id", "sim_ned")
        self.declare_parameter("odom_child_frame_id", "sim_body")

        p = self.get_parameter
        conn_str = p("connection_string").value
        imu_topic = p("imu_topic").value
        self._imu_frame_id = p("imu_frame_id").value
        heartbeat_period = p("heartbeat_period_s").value
        setpoint_topic = p("setpoint_topic").value
        auto_arm = p("auto_offboard_arm").value
        self._odom_frame = p("odom_frame_id").value
        self._odom_child = p("odom_child_frame_id").value

        log = self.get_logger().info

        # ----- core pieces -----
        self._aligner = TimeAligner(log=log)
        self._tx = MavlinkTransport(conn_str, log=log)
        self._control = OffboardController(self._tx, auto_arm, log=log)

        # ----- publishers -----
        self._imu_pub = self.create_publisher(Imu, imu_topic, 50)
        self._odom_pub = self.create_publisher(Odometry, "/sim/odometry", 10)
        self._pose_pub = self.create_publisher(
            PoseStamped, "/sim/local_position", 10
        )

        # ----- competition publishers -----
        _transient = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        _transient1 = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._race_status_pub = self.create_publisher(RaceStatus, "/race/status", _transient)
        self._track_pub = self.create_publisher(TrackData, "/race/track", _transient1)
        self._collision_pub = self.create_publisher(Collision, "/sim/collision", 10)
        self._attitude_pub = self.create_publisher(Vector3Stamped, "/sim/attitude", 10)
        self._motor_rpms_pub = self.create_publisher(Float32MultiArray, "/sim/motor_rpms", 10)

        # ----- track data chunk assembly state -----
        self._track_chunks: dict[int, dict[int, bytes]] = {}
        self._track_expected: dict[int, int] = {}

        # ----- subscriber: latest setpoint only -----
        sp_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            AttitudeTarget, setpoint_topic, self._on_attitude_target, sp_qos
        )

        # ----- register transport handlers -----
        self._tx.register("HIGHRES_IMU", self._on_highres_imu)
        self._tx.register("ODOMETRY", self._on_odometry)
        self._tx.register("LOCAL_POSITION_NED", self._on_local_position)
        self._tx.register("ATTITUDE", self._on_attitude)
        self._tx.register("ENCAPSULATED_DATA", self._on_encapsulated_data)
        self._tx.register("DATA_TRANSMISSION_HANDSHAKE", self._on_track_handshake)
        self._tx.register("COLLISION", self._on_collision)
        self._tx.register("ACTUATOR_OUTPUT_STATUS", self._on_actuator_output)

        # ----- timers -----
        self.create_timer(0.001, lambda: self._tx.poll(max_messages=64))
        self.create_timer(heartbeat_period, self._tx.send_heartbeat)
        self.create_timer(0.1, self._tx.send_timesync_request)
        self.create_timer(5.0, self._log_stats)

        # ----- counters -----
        self._imu_published = 0
        self._raw_imu_dumped = 0
        self._setpoints_forwarded = 0
        self._last_imu_time_us: int = -1

    # ---------------------------------------------------------------- #
    # MAVLink handlers (called by transport.poll)
    # ---------------------------------------------------------------- #
    def _on_highres_imu(self, msg) -> None:
        if msg.time_usec == self._last_imu_time_us:
            return
        self._last_imu_time_us = msg.time_usec
        self._maybe_dump_raw_imu(msg)
        imu = converters.highres_imu_to_imu(msg, self._aligner, self._imu_frame_id)
        self._imu_pub.publish(imu)
        self._imu_published += 1

    def _on_odometry(self, msg) -> None:
        stamp = self.get_clock().now().to_msg()
        odom = converters.odometry_to_odometry(
            msg, stamp, self._odom_frame, self._odom_child
        )
        self._odom_pub.publish(odom)

    def _on_local_position(self, msg) -> None:
        stamp = self.get_clock().now().to_msg()
        pose = converters.local_position_to_pose(msg, stamp, self._odom_frame)
        self._pose_pub.publish(pose)

    # ---------------------------------------------------------------- #
    # Competition MAVLink handlers
    # ---------------------------------------------------------------- #
    def _on_attitude(self, msg) -> None:
        self._attitude_pub.publish(converters.attitude_to_vector(msg, self._aligner))

    def _on_collision(self, msg) -> None:
        stamp = self.get_clock().now().to_msg()
        self._collision_pub.publish(converters.collision_from_mavlink(msg, stamp))

    def _on_actuator_output(self, msg) -> None:
        out = Float32MultiArray()
        out.data = [float(msg.actuator[i]) for i in range(4)]
        self._motor_rpms_pub.publish(out)

    def _on_track_handshake(self, msg) -> None:
        tid = msg.width
        self._track_chunks[tid] = {}
        self._track_expected[tid] = msg.packets

    def _on_encapsulated_data(self, msg) -> None:
        raw = bytes(msg.data)
        if not raw:
            return
        data_type = raw[0]
        stamp = self.get_clock().now().to_msg()
        if data_type == 1:
            self._race_status_pub.publish(converters.race_status_from_bytes(raw, stamp))
        elif data_type == 2:
            _, transfer_id = struct.unpack_from("<BH", raw)
            if transfer_id not in self._track_expected:
                return
            self._track_chunks[transfer_id][msg.seqnr] = raw[3:]
            expected = self._track_expected[transfer_id]
            if len(self._track_chunks[transfer_id]) == expected:
                payload = b"".join(
                    self._track_chunks[transfer_id][i] for i in range(expected)
                )
                del self._track_chunks[transfer_id]
                del self._track_expected[transfer_id]
                self._track_pub.publish(converters.track_data_from_bytes(payload, stamp))

    # ---------------------------------------------------------------- #
    # Setpoint subscriber
    # ---------------------------------------------------------------- #
    def _on_attitude_target(self, msg: AttitudeTarget) -> None:
        if self._control.forward_attitude_target(msg):
            self._setpoints_forwarded += 1

    # ---------------------------------------------------------------- #
    # Diagnostics
    # ---------------------------------------------------------------- #
    def _maybe_dump_raw_imu(self, msg) -> None:
        # Dump the first 10 raw samples to read at-rest accel/gyro directly.
        # At rest |accel| ~9.81 (m/s^2) with gravity on one axis; if ~1.0 the
        # sim sends g (set frames.ACCEL_IS_IN_G). Gyro ~0 on all axes.
        if self._raw_imu_dumped >= 10:
            return
        self._raw_imu_dumped += 1
        from . import frames
        a = frames.imu_accel_body(msg.xacc, msg.yacc, msg.zacc)
        g = frames.imu_gyro_body(msg.xgyro, msg.ygyro, msg.zgyro)
        self.get_logger().info(
            f"[RAW-IMU #{self._raw_imu_dumped}] "
            f"accel=({a.x:+.4f},{a.y:+.4f},{a.z:+.4f}) "
            f"|a|={frames.accel_magnitude(a):.4f} | "
            f"gyro=({g.x:+.4f},{g.y:+.4f},{g.z:+.4f}) "
            f"|g|={frames.accel_magnitude(g):.4f}"
        )

    def _log_stats(self) -> None:
        if self._imu_published == 0:
            self.get_logger().warn(
                f"No HIGHRES_IMU published yet. "
                f"heartbeats={self._tx.heartbeats_received} "
                f"unknown={self._tx.unknown_messages} "
                f"peer_seen={self._tx.peer_seen}"
            )
            return
        self.get_logger().info(
            f"imu_published={self._imu_published} "
            f"heartbeats={self._tx.heartbeats_received} "
            f"setpoints_fwd={self._setpoints_forwarded} "
            f"unknown={self._tx.unknown_messages}"
        )


def main() -> None:
    rclpy.init()
    node = MavBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._tx.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()