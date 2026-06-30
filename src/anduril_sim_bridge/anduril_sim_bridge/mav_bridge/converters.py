"""
converters.py: pure functions mapping MAVLink messages to ROS messages.

No sockets, no ROS node, no rclpy clock reads inside the IMU path - the only
inputs are the MAVLink message and a TimeAligner. This makes every converter
unit-testable with a synthetic message and an assert on the output, the same
self-test discipline used for the planner.

The odometry/local-position converters DO take a wall-clock stamp argument
rather than reading a clock internally, again to keep them pure. The node
passes the stamp in.
"""

from __future__ import annotations

import struct
from typing import Optional

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from anduril_msgs.msg import RaceStatus, Gate, TrackData, Collision

from . import frames
from .time_align import TimeAligner

# Stream names used for the TimeAligner origin cross-check.
STREAM_IMU = "imu"
STREAM_CAMERA = "camera"  # camera bridge registers this name


def highres_imu_to_imu(
        msg,
        aligner: TimeAligner,
        frame_id: str,
) -> Imu:
    """
    Convert HIGHRES_IMU -> sensor_msgs/Imu.

    Time: stamped on the shared sim timebase via the aligner (sim microseconds).
    Axes: routed through frames.py so a convention change is one edit there.
    Orientation: marked "no estimate" per REP-145 (covariance[0] = -1); the
    sim's HIGHRES_IMU carries no usable orientation and feeding the autopilot's
    attitude back into the attitude estimator would be a feedback loop.
    """
    imu = Imu()

    sec, nsec = aligner.stamp_from_us(msg.time_usec, STREAM_IMU)
    imu.header.stamp.sec = sec
    imu.header.stamp.nanosec = nsec
    imu.header.frame_id = frame_id

    imu.orientation.w = 1.0
    imu.orientation_covariance[0] = -1.0

    g = frames.imu_gyro_body(msg.xgyro, msg.ygyro, msg.zgyro)
    a = frames.imu_accel_body(msg.xacc, msg.yacc, msg.zacc)
    imu.angular_velocity.x = g.x
    imu.angular_velocity.y = g.y
    imu.angular_velocity.z = g.z
    imu.linear_acceleration.x = a.x
    imu.linear_acceleration.y = a.y
    imu.linear_acceleration.z = a.z

    return imu


def odometry_to_odometry(msg, stamp, frame_id: str, child_frame_id: str) -> Odometry:
    """
    Convert MAVLink ODOMETRY -> nav_msgs/Odometry.
    `stamp` is a builtin_interfaces/Time supplied by the caller (kept pure).
    Sim sends quaternion as [w, x, y, z].
    """
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = frame_id
    odom.child_frame_id = child_frame_id

    odom.pose.pose.position.x = float(msg.x)
    odom.pose.pose.position.y = float(msg.y)
    odom.pose.pose.position.z = float(msg.z)

    odom.pose.pose.orientation.w = float(msg.q[0])
    odom.pose.pose.orientation.x = float(msg.q[1])
    odom.pose.pose.orientation.y = float(msg.q[2])
    odom.pose.pose.orientation.z = float(msg.q[3])

    odom.twist.twist.linear.x = float(msg.vx)
    odom.twist.twist.linear.y = float(msg.vy)
    odom.twist.twist.linear.z = float(msg.vz)
    odom.twist.twist.angular.x = float(msg.rollspeed)
    odom.twist.twist.angular.y = float(msg.pitchspeed)
    odom.twist.twist.angular.z = float(msg.yawspeed)

    return odom


def local_position_to_pose(msg, stamp, frame_id: str) -> PoseStamped:
    """Convert MAVLink LOCAL_POSITION_NED -> geometry_msgs/PoseStamped."""
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(msg.x)
    pose.pose.position.y = float(msg.y)
    pose.pose.position.z = float(msg.z)
    return pose


# ---------------------------------------------------------------------------
# Competition-specific converters
# ---------------------------------------------------------------------------

# ENCAPSULATED_DATA sub-message ids
_RACE_STATUS_ID = 1
_TRACK_DATA_ID = 2

# Struct formats matching the competition wire protocol
_RACE_STATUS_FMT = "<BQqqIq"   # data_type + 5 fields
_GATE_FMT = "<Hfffffffff"      # gate_id + 9 floats
_GATE_SIZE = struct.calcsize(_GATE_FMT)   # 38 bytes


def race_status_from_bytes(raw: bytes, stamp) -> RaceStatus:
    """Parse an ENCAPSULATED_DATA race-status payload -> RaceStatus msg."""
    _, sim_boot_ms, race_start, race_finish, active_gate, last_gate_time = \
        struct.unpack_from(_RACE_STATUS_FMT, raw)
    msg = RaceStatus()
    msg.header.stamp = stamp
    msg.sim_boot_time_ms = sim_boot_ms
    msg.race_start_boot_time_ms = race_start
    msg.race_finish_time_ns = race_finish
    msg.active_gate_index = active_gate
    msg.last_gate_race_time_ns = last_gate_time
    return msg


def track_data_from_bytes(payload: bytes, stamp) -> TrackData:
    """Parse a fully-assembled ENCAPSULATED_DATA track payload -> TrackData msg."""
    num_gates, = struct.unpack_from("<H", payload)
    offset = 2
    msg = TrackData()
    msg.header.stamp = stamp
    for _ in range(num_gates):
        gate_id, px, py, pz, qw, qx, qy, qz, width, height = \
            struct.unpack_from(_GATE_FMT, payload, offset)
        offset += _GATE_SIZE
        gate = Gate()
        gate.gate_id = gate_id
        gate.position_ned.x = float(px)
        gate.position_ned.y = float(py)
        gate.position_ned.z = float(pz)
        gate.orientation_ned.w = float(qw)
        gate.orientation_ned.x = float(qx)
        gate.orientation_ned.y = float(qy)
        gate.orientation_ned.z = float(qz)
        gate.width = float(width)
        gate.height = float(height)
        msg.gates.append(gate)
    return msg


def collision_from_mavlink(msg, stamp) -> Collision:
    """Convert MAVLink COLLISION -> anduril_msgs/Collision."""
    out = Collision()
    out.header.stamp = stamp
    out.collision_id = int(msg.id)
    out.threat_level = int(msg.threat_level)
    out.impulse = float(msg.horizontal_minimum_delta)
    return out


def attitude_to_vector(msg, aligner: TimeAligner) -> Vector3Stamped:
    """Convert MAVLink ATTITUDE -> geometry_msgs/Vector3Stamped (roll/pitch/yaw rad)."""
    out = Vector3Stamped()
    sec, nsec = aligner.stamp_from_us(msg.time_boot_ms * 1000, "attitude")
    out.header.stamp.sec = sec
    out.header.stamp.nanosec = nsec
    out.vector.x = float(msg.roll)
    out.vector.y = float(msg.pitch)
    out.vector.z = float(msg.yaw)
    return out