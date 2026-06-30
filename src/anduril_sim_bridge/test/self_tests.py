"""
Self-tests for the pure modules. No ROS, no socket, no sim required.

We stub the ROS message classes and the MAVLink messages so converters can run
standalone. Run: python3 selftest.py
"""

import sys
import types


# --------------------------------------------------------------------- #
# Stub the ROS message packages so converters.py imports cleanly without ROS.
# --------------------------------------------------------------------- #
def _make_stub_msgs():
    class _Stamp:
        def __init__(self): self.sec = 0; self.nanosec = 0
    class _Header:
        def __init__(self): self.stamp = _Stamp(); self.frame_id = ""
    class _V3:
        def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0
    class _Quat:
        def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0; self.w = 0.0
    class _Point:
        def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0

    class Imu:
        def __init__(self):
            self.header = _Header()
            self.orientation = _Quat()
            self.orientation_covariance = [0.0] * 9
            self.angular_velocity = _V3()
            self.linear_acceleration = _V3()

    class _Pose:
        def __init__(self): self.position = _Point(); self.orientation = _Quat()
    class _PoseWrap:
        def __init__(self): self.pose = _Pose()
    class _TwistInner:
        def __init__(self): self.linear = _V3(); self.angular = _V3()
    class _TwistWrap:
        def __init__(self): self.twist = _TwistInner()

    class Odometry:
        def __init__(self):
            self.header = _Header(); self.child_frame_id = ""
            self.pose = _PoseWrap(); self.twist = _TwistWrap()

    class PoseStamped:
        def __init__(self):
            self.header = _Header(); self.pose = _Pose()

    class Vector3Stamped:
        def __init__(self): self.header = _Header(); self.vector = _V3()

    class RaceStatus:
        def __init__(self):
            self.header = _Header()
            self.sim_boot_time_ms = 0
            self.race_start_boot_time_ms = 0
            self.race_finish_time_ns = 0
            self.active_gate_index = 0
            self.last_gate_race_time_ns = 0

    class Gate:
        def __init__(self):
            self.gate_id = 0
            self.position_ned = _Point()
            self.orientation_ned = _Quat()
            self.width = 0.0
            self.height = 0.0

    class TrackData:
        def __init__(self): self.header = _Header(); self.gates = []

    class Collision:
        def __init__(self):
            self.header = _Header()
            self.collision_id = 0
            self.threat_level = 0
            self.impulse = 0.0

    sensor = types.ModuleType("sensor_msgs.msg"); sensor.Imu = Imu
    nav = types.ModuleType("nav_msgs.msg"); nav.Odometry = Odometry
    geo = types.ModuleType("geometry_msgs.msg")
    geo.PoseStamped = PoseStamped
    geo.Vector3Stamped = Vector3Stamped
    anduril_msgs_msg = types.ModuleType("anduril_msgs.msg")
    anduril_msgs_msg.RaceStatus = RaceStatus
    anduril_msgs_msg.Gate = Gate
    anduril_msgs_msg.TrackData = TrackData
    anduril_msgs_msg.Collision = Collision
    sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
    sys.modules["sensor_msgs.msg"] = sensor
    sys.modules["nav_msgs"] = types.ModuleType("nav_msgs")
    sys.modules["nav_msgs.msg"] = nav
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = geo
    sys.modules["anduril_msgs"] = types.ModuleType("anduril_msgs")
    sys.modules["anduril_msgs.msg"] = anduril_msgs_msg


_make_stub_msgs()

from anduril_sim_bridge.mav_bridge.time_align import TimeAligner, NS_PER_SEC
from anduril_sim_bridge.mav_bridge import frames
from anduril_sim_bridge.mav_bridge import converters


class FakeImuMsg:
    def __init__(self, t_us, accel, gyro):
        self.time_usec = t_us
        self.xacc, self.yacc, self.zacc = accel
        self.xgyro, self.ygyro, self.zgyro = gyro


class FakeOdomMsg:
    def __init__(self):
        self.x, self.y, self.z = 1.0, 2.0, 3.0
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.vx, self.vy, self.vz = 0.1, 0.2, 0.3
        self.rollspeed, self.pitchspeed, self.yawspeed = 0.01, 0.02, 0.03


def t(name, cond):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
    if not cond:
        raise AssertionError(name)


def _to_ns(sec, nsec):
    return sec * NS_PER_SEC + nsec


def test_aligner_conversion():
    print("TimeAligner: us/ns conversion preserves relative timing")
    a = TimeAligner()
    # Both stamps use the same epoch offset; only the delta matters.
    sec1, nsec1 = a.stamp_from_us(1_500_000, "imu")   # sim 1.5 s
    sec2, nsec2 = a.stamp_from_ns(2 * NS_PER_SEC + 250, "imu")  # sim 2.000000250 s
    delta_ns = _to_ns(sec2, nsec2) - _to_ns(sec1, nsec1)
    expected = NS_PER_SEC // 2 + 250  # 0.5 s + 250 ns
    t("delta is 0.5 s + 250 ns", delta_ns == expected)
    # Output should be near current wall time (epoch offset applied).
    import time
    now_ns = time.time_ns()
    t("stamps near wall time", abs(_to_ns(sec1, nsec1) - now_ns) < 5 * NS_PER_SEC)


def test_aligner_cross_check():
    print("TimeAligner: origin cross-check fires once with >=2 streams")
    logs = []
    a = TimeAligner(log=logs.append, warn_threshold_ns=100_000_000)
    a.stamp_from_us(1_000_000, "imu")        # sim 1.0 s
    a.stamp_from_ns(1_010_000_000, "camera")  # sim 1.01 s -> 10 ms delta, shared
    fired = [m for m in logs if "cross-check" in m]
    t("cross-check logged once", len(fired) == 1)
    t("reports shared origin", "share a clock origin" in fired[0])
    # Further samples must not re-fire.
    a.stamp_from_us(2_000_000, "imu")
    fired2 = [m for m in logs if "cross-check" in m]
    t("does not re-fire", len(fired2) == 1)


def test_aligner_cross_check_large_delta():
    print("TimeAligner: large origin delta warns")
    logs = []
    a = TimeAligner(log=logs.append, warn_threshold_ns=100_000_000)
    a.stamp_from_us(1_000_000, "imu")         # sim 1.0 s
    a.stamp_from_ns(5_000_000_000, "camera")   # sim 5.0 s -> 4 s delta -> LARGE
    warned = [m for m in logs if "LARGE" in m]
    t("large delta warned", len(warned) == 1)


def test_aligner_offset():
    print("TimeAligner: set_offset shifts one stream relative to others")
    a = TimeAligner()
    # Both streams given identical sim time; per-stream offset shifts one by -1 s.
    a.set_offset("camera", -1 * NS_PER_SEC)
    sec_cam, nsec_cam = a.stamp_from_ns(5 * NS_PER_SEC, "camera")
    sec_imu, nsec_imu = a.stamp_from_us(5_000_000, "imu")
    cam_ns = _to_ns(sec_cam, nsec_cam)
    imu_ns = _to_ns(sec_imu, nsec_imu)
    t("camera is 1 s behind imu at same sim time", imu_ns - cam_ns == NS_PER_SEC)


def test_frames_identity_and_scale():
    print("frames: identity mapping, g scaling toggle")
    g = frames.imu_gyro_body(0.1, -0.2, 0.3)
    t("gyro identity", (g.x, g.y, g.z) == (0.1, -0.2, 0.3))
    frames.ACCEL_IS_IN_G = False
    a = frames.imu_accel_body(0.0, 0.0, 9.81)
    t("accel no-scale", abs(a.z - 9.81) < 1e-9)
    frames.ACCEL_IS_IN_G = True
    a2 = frames.imu_accel_body(0.0, 0.0, 1.0)
    t("accel g->m/s^2", abs(a2.z - 9.80665) < 1e-9)
    frames.ACCEL_IS_IN_G = False  # restore


def test_imu_converter():
    print("converters: HIGHRES_IMU -> Imu, stamped on shared timebase")
    import time
    a = TimeAligner()
    msg1 = FakeImuMsg(t_us=3_000_000, accel=(0.0, 0.0, 9.81), gyro=(0.0, 0.0, 0.0))
    msg2 = FakeImuMsg(t_us=3_500_000, accel=(0.0, 0.0, 9.81), gyro=(0.0, 0.0, 0.0))
    before_ns = time.time_ns()
    imu1 = converters.highres_imu_to_imu(msg1, a, "imu0")
    imu2 = converters.highres_imu_to_imu(msg2, a, "imu0")
    after_ns = time.time_ns()
    # Stamp should land on Unix epoch (within a small window around call time).
    stamp1_ns = imu1.header.stamp.sec * NS_PER_SEC + imu1.header.stamp.nanosec
    t("stamp near wall time", before_ns - NS_PER_SEC <= stamp1_ns <= after_ns + NS_PER_SEC)
    # Delta between consecutive messages must equal the sim-time delta (0.5 s).
    stamp2_ns = imu2.header.stamp.sec * NS_PER_SEC + imu2.header.stamp.nanosec
    t("delta = 0.5 s", stamp2_ns - stamp1_ns == 500_000_000)
    t("frame_id", imu1.header.frame_id == "imu0")
    t("orientation marked no-estimate", imu1.orientation_covariance[0] == -1.0)
    t("accel z passthrough", abs(imu1.linear_acceleration.z - 9.81) < 1e-9)


def test_odom_converter():
    print("converters: ODOMETRY -> Odometry, q order [w,x,y,z]")
    msg = FakeOdomMsg()
    stamp = object()  # caller-supplied; converter must not introspect it
    odom = converters.odometry_to_odometry(msg, stamp, "sim_ned", "sim_body")
    t("position x", odom.pose.pose.position.x == 1.0)
    t("quat w from q[0]", odom.pose.pose.orientation.w == 1.0)
    t("twist linear x", odom.twist.twist.linear.x == 0.1)
    t("child frame", odom.child_frame_id == "sim_body")


def test_race_status_parse():
    print("converters: race_status_from_bytes parses known payload")
    import struct
    raw = struct.pack("<BQqqIq", 1, 12345, 1000, -1, 3, 9876)
    stamp = object()
    rs = converters.race_status_from_bytes(raw, stamp)
    t("sim_boot_time_ms", rs.sim_boot_time_ms == 12345)
    t("race_start_boot_time_ms", rs.race_start_boot_time_ms == 1000)
    t("race_finish_time_ns == -1", rs.race_finish_time_ns == -1)
    t("active_gate_index", rs.active_gate_index == 3)
    t("last_gate_race_time_ns", rs.last_gate_race_time_ns == 9876)


def test_track_data_parse():
    print("converters: track_data_from_bytes parses single gate")
    import struct
    payload = struct.pack("<H", 1)   # num_gates = 1
    payload += struct.pack("<Hfffffffff", 7, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0, 2.5, 2.0)
    stamp = object()
    td = converters.track_data_from_bytes(payload, stamp)
    t("gate count", len(td.gates) == 1)
    t("gate_id", td.gates[0].gate_id == 7)
    t("position_ned x", abs(td.gates[0].position_ned.x - 1.0) < 1e-6)
    t("position_ned y", abs(td.gates[0].position_ned.y - 2.0) < 1e-6)
    t("width", abs(td.gates[0].width - 2.5) < 1e-6)
    t("height", abs(td.gates[0].height - 2.0) < 1e-6)
    t("orient w", abs(td.gates[0].orientation_ned.w - 1.0) < 1e-6)


def main():
    tests = [
        test_aligner_conversion,
        test_aligner_cross_check,
        test_aligner_cross_check_large_delta,
        test_aligner_offset,
        test_frames_identity_and_scale,
        test_imu_converter,
        test_odom_converter,
        test_race_status_parse,
        test_track_data_parse,
    ]
    for fn in tests:
        fn()
    print("\nAll self-tests passed.")


if __name__ == "__main__":
    main()