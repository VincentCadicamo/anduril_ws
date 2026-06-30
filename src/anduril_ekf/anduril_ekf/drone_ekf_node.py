"""
drone_ekf_node.py — ROS2 wrapper around DroneEKF.

Subscriptions
-------------
  /imu0              sensor_msgs/Imu                  IMU propagation
  /gate/pose         geometry_msgs/PoseWithCovarianceStamped  gate update
  /race/track        anduril_msgs/TrackData            gate world positions
  /race/status       anduril_msgs/RaceStatus           active gate index
  /sim/odometry      nav_msgs/Odometry                 sim ground-truth init

Publications
------------
  /drone/odom        nav_msgs/Odometry                 EKF fused estimate
"""

from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from anduril_msgs.msg import TrackData, RaceStatus

from .ekf import DroneEKF


_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Matches the TRANSIENT_LOCAL profile used by mav_bridge for /race/track and
# /race/status so late-joining subscribers still receive the last message.
_TRANSIENT_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class DroneEKFNode(Node):
    """Fuses IMU + PnP gate detections via an error-state EKF."""

    def __init__(self) -> None:
        super().__init__("drone_ekf_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("sigma_acc",       0.3)
        self.declare_parameter("sigma_gyr",       0.02)
        self.declare_parameter("sigma_ba_rw",     0.002)
        self.declare_parameter("sigma_bg_rw",     2e-4)
        self.declare_parameter("chi2_thresh",     11.345)
        self.declare_parameter("imu_topic",       "/imu0")
        self.declare_parameter("gate_pose_topic", "/gate/pose")
        self.declare_parameter("odom_out_topic",  "/drone/odom")
        # ZUPT parameters
        self.declare_parameter("zupt_sigma_v",      0.02)   # m/s — velocity noise when static
        self.declare_parameter("zupt_omega_thresh",  0.05)  # rad/s — gyro magnitude gate
        self.declare_parameter("zupt_accel_thresh",  0.5)   # m/s² — |f|−g magnitude gate

        p = self.get_parameter
        self._ekf = DroneEKF(
            sigma_acc   = float(p("sigma_acc").value),
            sigma_gyr   = float(p("sigma_gyr").value),
            sigma_ba_rw = float(p("sigma_ba_rw").value),
            sigma_bg_rw = float(p("sigma_bg_rw").value),
            chi2_thresh = float(p("chi2_thresh").value),
        )

        imu_topic       = p("imu_topic").value
        gate_pose_topic = p("gate_pose_topic").value
        odom_out_topic  = p("odom_out_topic").value
        self._zupt_sigma_v     = float(p("zupt_sigma_v").value)
        self._zupt_omega_thresh = float(p("zupt_omega_thresh").value)
        self._zupt_accel_thresh = float(p("zupt_accel_thresh").value)

        # ── State ─────────────────────────────────────────────────────────────
        self._last_imu_stamp: float | None = None
        self._track_data:  TrackData | None = None
        self._active_gate_idx: int = 0
        self._sim_init_done: bool = False
        self._gate_updates: int = 0
        self._gate_rejects: int = 0

        # ── TF broadcaster ───────────────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── Publishers ────────────────────────────────────────────────────────
        self._odom_pub         = self.create_publisher(Odometry,     odom_out_topic,    10)
        self._path_pub         = self.create_publisher(Path,         "/drone/path",     10)
        self._gate_markers_pub = self.create_publisher(MarkerArray,  "/gate/markers",   10)

        self._path_msg = Path()
        self._path_msg.header.frame_id = "world_ned"
        self._path_max = 3000  # ~25 s at 120 Hz

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Imu, imu_topic, self._on_imu, _BEST_EFFORT_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped,
            gate_pose_topic,
            self._on_gate_pose,
            10,
        )
        self.create_subscription(
            TrackData, "/race/track", self._on_track_data, _TRANSIENT_QOS
        )
        self.create_subscription(
            RaceStatus, "/race/status", self._on_race_status, _TRANSIENT_QOS
        )
        self.create_subscription(
            Odometry,
            "/sim/odometry",
            self._on_sim_odometry,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            ),
        )

        self.create_timer(5.0, self._log_stats)
        self.get_logger().info(
            f"DroneEKFNode started — IMU: {imu_topic}, "
            f"gate: {gate_pose_topic}, out: {odom_out_topic}"
        )

    # ── Initialization ────────────────────────────────────────────────────────

    def _on_sim_odometry(self, msg: Odometry) -> None:
        """Seed the EKF from sim ground-truth on the first sample."""
        if self._sim_init_done:
            return
        p = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        q = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])
        self._ekf.initialize(p, q)
        self._sim_init_done = True
        self.get_logger().info(
            f"EKF initialized from sim odometry: p={p.round(2)}"
        )

    # ── IMU propagation ───────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_imu_stamp is None:
            self._last_imu_stamp = t
            return
        dt = t - self._last_imu_stamp
        self._last_imu_stamp = t

        if dt <= 0.0 or dt > 0.5:
            return

        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        f = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        self._ekf.propagate(omega, f, dt)

        if self._is_stationary(omega, f):
            self._ekf.update_zupt(self._zupt_sigma_v)

        if self._ekf.initialized:
            self._publish_odom(msg.header.stamp)

    def _is_stationary(self, omega: np.ndarray, f: np.ndarray) -> bool:
        """True when gyro and net specific force are both near zero."""
        return (
            float(np.linalg.norm(omega)) < self._zupt_omega_thresh
            and abs(float(np.linalg.norm(f)) - 9.80665) < self._zupt_accel_thresh
        )

    # ── Gate measurement update ───────────────────────────────────────────────

    def _on_gate_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._ekf.initialized or self._track_data is None:
            return
        gates = self._track_data.gates
        idx   = self._active_gate_idx
        if idx >= len(gates):
            return

        gate = gates[idx]
        gate_pos_W = np.array([
            gate.position_ned.x,
            gate.position_ned.y,
            gate.position_ned.z,
        ])

        z_flu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

        # Extract 3×3 position covariance from the 6×6 row-major array
        c = msg.pose.covariance
        R_meas = np.array([
            [c[0],  c[1],  c[2] ],
            [c[6],  c[7],  c[8] ],
            [c[12], c[13], c[14]],
        ])

        accepted, innov, mahal = self._ekf.update_gate(z_flu, R_meas, gate_pos_W)
        if accepted:
            self._gate_updates += 1
        else:
            self._gate_rejects += 1
            self.get_logger().debug(
                f"Gate update rejected (mahal={mahal:.2f} > "
                f"{self._ekf.chi2_thresh:.2f})"
            )

    # ── Competition data ──────────────────────────────────────────────────────

    def _on_track_data(self, msg: TrackData) -> None:
        self._track_data = msg
        self.get_logger().info(f"Track data received: {len(msg.gates)} gates")
        self._publish_gate_markers()

    def _on_race_status(self, msg: RaceStatus) -> None:
        self._active_gate_idx = int(msg.active_gate_index)
        if self._track_data is not None:
            self._publish_gate_markers()

    def _publish_gate_markers(self) -> None:
        """Publish gate world positions as RViz markers (active gate highlighted)."""
        if self._track_data is None:
            return
        now = self.get_clock().now().to_msg()
        ma  = MarkerArray()
        for i, gate in enumerate(self._track_data.gates):
            active = (i == self._active_gate_idx)
            m = Marker()
            m.header.frame_id = "world_ned"
            m.header.stamp    = now
            m.ns              = "gates"
            m.id              = i
            m.type            = Marker.CUBE
            m.action          = Marker.ADD
            m.pose.position.x = float(gate.position_ned.x)
            m.pose.position.y = float(gate.position_ned.y)
            m.pose.position.z = float(gate.position_ned.z)
            m.pose.orientation.w = float(gate.orientation_ned.w)
            m.pose.orientation.x = float(gate.orientation_ned.x)
            m.pose.orientation.y = float(gate.orientation_ned.y)
            m.pose.orientation.z = float(gate.orientation_ned.z)
            m.scale.x = float(gate.width)
            m.scale.y = 0.15
            m.scale.z = float(gate.height)
            if active:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 1.0, 0.4
            ma.markers.append(m)

            # Gate number label
            lbl = Marker()
            lbl.header.frame_id = "world_ned"
            lbl.header.stamp    = now
            lbl.ns              = "gate_labels"
            lbl.id              = i
            lbl.type            = Marker.TEXT_VIEW_FACING
            lbl.action          = Marker.ADD
            lbl.pose.position.x = float(gate.position_ned.x)
            lbl.pose.position.y = float(gate.position_ned.y)
            lbl.pose.position.z = float(gate.position_ned.z) - float(gate.height) / 2.0 - 0.5
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 1.0
            lbl.color.r, lbl.color.g, lbl.color.b, lbl.color.a = 1.0, 1.0, 1.0, 1.0
            lbl.text = f"Gate {i}"
            ma.markers.append(lbl)

        self._gate_markers_pub.publish(ma)

    # ── Output ────────────────────────────────────────────────────────────────

    def _publish_odom(self, stamp) -> None:
        odom = Odometry()
        odom.header.stamp      = stamp
        odom.header.frame_id   = "world_ned"
        odom.child_frame_id    = "body"
        odom.pose.pose.position.x    = float(self._ekf.p[0])
        odom.pose.pose.position.y    = float(self._ekf.p[1])
        odom.pose.pose.position.z    = float(self._ekf.p[2])
        odom.pose.pose.orientation.w = float(self._ekf.q[0])
        odom.pose.pose.orientation.x = float(self._ekf.q[1])
        odom.pose.pose.orientation.y = float(self._ekf.q[2])
        odom.pose.pose.orientation.z = float(self._ekf.q[3])
        odom.twist.twist.linear.x    = float(self._ekf.v[0])
        odom.twist.twist.linear.y    = float(self._ekf.v[1])
        odom.twist.twist.linear.z    = float(self._ekf.v[2])

        # Populate 6×6 pose covariance diagonal from P (position + attitude)
        P = self._ekf.P
        for i in range(3):
            odom.pose.covariance[i * 7]       = float(P[i,     i    ])  # pos
            odom.pose.covariance[(i + 3) * 7] = float(P[i + 6, i + 6])  # att

        self._odom_pub.publish(odom)

        # Broadcast world_ned → body TF so RViz fixed frame resolves
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = "world_ned"
        tf.child_frame_id  = "body"
        tf.transform.translation.x = float(self._ekf.p[0])
        tf.transform.translation.y = float(self._ekf.p[1])
        tf.transform.translation.z = float(self._ekf.p[2])
        tf.transform.rotation.w = float(self._ekf.q[0])
        tf.transform.rotation.x = float(self._ekf.q[1])
        tf.transform.rotation.y = float(self._ekf.q[2])
        tf.transform.rotation.z = float(self._ekf.q[3])
        self._tf_broadcaster.sendTransform(tf)

        # Append to path (cap length to avoid unbounded memory)
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose   = odom.pose.pose
        self._path_msg.poses.append(ps)
        if len(self._path_msg.poses) > self._path_max:
            self._path_msg.poses.pop(0)
        self._path_msg.header.stamp = stamp
        self._path_pub.publish(self._path_msg)

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def _log_stats(self) -> None:
        init_str = "INIT" if self._ekf.initialized else "WAITING"
        self.get_logger().info(
            f"[EKF {init_str}] gate_updates={self._gate_updates} "
            f"gate_rejects={self._gate_rejects} "
            f"active_gate={self._active_gate_idx}"
        )
        if self._ekf.initialized:
            p = self._ekf.p
            self.get_logger().info(
                f"[EKF pos] N={p[0]:.2f} E={p[1]:.2f} D={p[2]:.2f} m"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneEKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
