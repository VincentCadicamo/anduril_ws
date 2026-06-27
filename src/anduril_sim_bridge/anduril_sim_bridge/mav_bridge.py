"""
MAVLink bridge: receives MAVLink v2 telemetry from the simulator over UDP
and republishes HIGHRES_IMU as sensor_msgs/Imu on /imu0.

Section 4.3, we consume:
  HIGHRES_IMU   gyro + accel for VIO  -> /imu0
  HEARTBEAT     sim's liveness signal -> logged only
  TIMESYNC      time sync handshake   -> reply
We also emit our own HEARTBEAT back so the sim considers us connected
(section 4.4 says minimum 2 Hz from the client).

We do NOT consume ATTITUDE for VIO, that's the autopilot's downstream
estimate, and feeding it into the same pipeline that estimates attitude
would create a feedback loop. ATTITUDE is logged at low rate only.

Outbound control: we subscribe to mavros_msgs/AttitudeTarget on
/setpoint/attitude_target (published by the manual_control node) and forward
each setpoint to the sim via set_attitude_target_send. This bridge is the
single MAVLink owner, so all sim-bound commands funnel through here.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from pymavlink import mavutil
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import AttitudeTarget
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


#MAVLink ids
MAVLINK_COMPONENT_ID = 191        #191 = MAV_COMP_ID_ONBOARD_COMPUTER
MAVLINK_SYSTEM_ID = 245           #must not collide with the sim's


class MavBridge(Node):
    def __init__(self):
        super().__init__("mav_bridge")

        self.declare_parameter("connection_string", "udpin:0.0.0.0:14550")
        self.declare_parameter("imu_topic", "/imu0")
        self.declare_parameter("imu_frame_id", "imu0")
        self.declare_parameter("heartbeat_period_s", 1.0)
        self.declare_parameter("setpoint_topic", "/setpoint/attitude_target")
        #Auto offboard+arm once the autopilot is seen. Set false to drive
        #mode/arming externally.
        self.declare_parameter("auto_offboard_arm", True)
        self._epoch_offset_ns = None
        self.odom_pub = self.create_publisher(Odometry, '/sim/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/sim/local_position', 10)

        conn_str = self.get_parameter("connection_string").value
        topic = self.get_parameter("imu_topic").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        heartbeat_period = self.get_parameter("heartbeat_period_s").value
        setpoint_topic = self.get_parameter("setpoint_topic").value
        self.auto_offboard_arm = self.get_parameter("auto_offboard_arm").value

        self.get_logger().info(f"Opening MAVLink connection: {conn_str}")
        self.conn = mavutil.mavlink_connection(
            conn_str,
            source_system=MAVLINK_SYSTEM_ID,
            source_component=MAVLINK_COMPONENT_ID,
            dialect="common",
        )

        self.imu_pub = self.create_publisher(Imu, topic, 50)

        #Setpoint subscriber. best-effort/depth-1 matches the manual_control
        #publisher: only the latest setpoint matters, never queue stale ones.
        sp_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.attitude_sub = self.create_subscription(
            AttitudeTarget,
            setpoint_topic,
            self._on_attitude_target,
            sp_qos,
        )

        #Bridge boot time, used for set_attitude_target's time_boot_ms field.
        self._boot_ms = int(time.time() * 1000)
        #Tracks whether we've already issued offboard+arm.
        self._offboard_armed = False

        #1 ms tick to drain the MAVLink receive queue. pymavlink's recv_match
        #is non-blocking when given a 0 timeout, so this doesn't busy-wait.
        self.create_timer(0.001, self._poll_mavlink)

        #Outbound heartbeat at the configured rate (section 4.4 minimum 2 Hz).
        self.create_timer(heartbeat_period, self._send_heartbeat)

        self.create_timer(5.0, self._log_stats)

        #Counters
        self.imu_received = 0
        self.imu_published = 0
        self.heartbeats_received = 0
        self.attitude_received = 0
        self.setpoints_forwarded = 0
        self.unknown_messages = 0
        self.last_sim_time_us: int | None = None

        self._wait_for_first_heartbeat(timeout_s=5.0)

    def _wait_for_first_heartbeat(self, timeout_s: float):
        self.get_logger().info("Waiting for first HEARTBEAT from sim...")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self.conn.recv_match(type="HEARTBEAT", blocking=False)
            if msg is not None:
                self.get_logger().info(
                    f"Sim connected (system_id={msg.get_srcSystem()}, "
                    f"component_id={msg.get_srcComponent()})"
                )
                self.heartbeats_received += 1
                return
            time.sleep(0.05)
        self.get_logger().warn(
            "No HEARTBEAT received within timeout. "
            "Bridge will still run; check the connection string and sim status."
        )

    def _poll_mavlink(self):
        for _ in range(64):
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return
            msg_type = msg.get_type()
            if msg_type == "HIGHRES_IMU":
                self._handle_highres_imu(msg)
            elif msg_type == "HEARTBEAT":
                self.heartbeats_received += 1
            elif msg_type == "TIMESYNC":
                self._handle_timesync(msg)
            elif msg_type == "ATTITUDE":
                self.attitude_received += 1
            elif msg_type == "BAD_DATA":
                self.unknown_messages += 1
            elif msg_type == "LOCAL_POSITION_NED":
                self._handle_local_position(msg)
            elif msg_type == "ODOMETRY":
                self._handle_odometry(msg)
            else:
                self.unknown_messages += 1

    def _handle_highres_imu(self, msg):
        self.imu_received += 1
        self.last_sim_time_us = msg.time_usec

        #RAW-IMU DEBUG: dump the first 10 raw samples so we can read at-rest
        #accel/gyro values directly. At rest, accel magnitude should be ~9.81
        #(m/s^2) with gravity on one axis; if it's ~1.0 the sim sends g, not
        #m/s^2. Gyro should be ~0 on all axes. Remove once units/frame
        #convention is confirmed.
        if self.imu_received <= 10:
            ax, ay, az = msg.xacc, msg.yacc, msg.zacc
            gx, gy, gz = msg.xgyro, msg.ygyro, msg.zgyro
            accel_mag = (ax * ax + ay * ay + az * az) ** 0.5
            gyro_mag = (gx * gx + gy * gy + gz * gz) ** 0.5
            self.get_logger().info(
                f"[RAW-IMU #{self.imu_received}] "
                f"accel=({ax:+.4f},{ay:+.4f},{az:+.4f}) |a|={accel_mag:.4f} | "
                f"gyro=({gx:+.4f},{gy:+.4f},{gz:+.4f}) |g|={gyro_mag:.4f}"
            )

        if self._epoch_offset_ns is None:
            wall_ns = time.time_ns()
            sim_ns = msg.time_usec * 1000
            self._epoch_offset_ns = wall_ns - sim_ns
            self.get_logger().info(
                f"Epoch offset (logging only, NOT applied to stamps): "
                f"{self._epoch_offset_ns / 1e9:.3f}s "
                f"(sim_time={msg.time_usec}us, wall={wall_ns}ns)"
            )


        imu = Imu()

        #Align the IMU clock to the camera bridge. The camera stamps frames
        #with sim_time_ns, which is Unix EPOCH time (confirmed: its stamps
        #track wall-clock). MAVLink HIGHRES_IMU.time_usec is autopilot
        #boot-relative time (~hundreds of seconds). Adding _epoch_offset_ns
        #(= wall_epoch - sim_boot, captured from the first sample) lifts the
        #IMU onto the same epoch timebase so OpenVINS can correlate IMU and
        #camera data. Both streams must share a clock or init never fires.
        stamp_ns = msg.time_usec * 1000 + self._epoch_offset_ns
        imu.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        imu.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        imu.header.frame_id = self.imu_frame_id

        #EuRoC convention: orientation is "no estimate" -> set covariance[0]
        #to -1 per REP-145. Downstream consumers must not read orientation.
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0

        #MAVLink HIGHRES_IMU body-frame conventions are FRD (forward-right-down),
        #matching MAVLink's BODY_NED. Section 3.8 says body-to-IMU is identity,
        imu.angular_velocity.x = float(msg.xgyro)
        imu.angular_velocity.y = float(msg.ygyro)
        imu.angular_velocity.z = float(msg.zgyro)
        imu.linear_acceleration.x = float(msg.xacc)
        imu.linear_acceleration.y = float(msg.yacc)
        imu.linear_acceleration.z = float(msg.zacc)

        self.imu_pub.publish(imu)
        self.imu_published += 1

    def _handle_timesync(self, msg):
        #Sim sent a TIMESYNC probe. Per the MAVLink convention, if tc1==0 the
        #sender wants us to fill it with our timestamp and echo it back
        if msg.tc1 == 0:
            self.conn.mav.timesync_send(
                tc1=time.time_ns(),
                ts1=msg.ts1,
            )

    def _send_heartbeat(self):
        #We're an onboard computer / companion NOT AN AUTOPILOT. Use
        #MAV_TYPE_ONBOARD_CONTROLLER (18) and MAV_AUTOPILOT_INVALID.
        self.conn.mav.heartbeat_send(
            type=18,                  #MAV_TYPE_ONBOARD_CONTROLLER
            autopilot=8,              #MAV_AUTOPILOT_INVALID
            base_mode=0,
            custom_mode=0,
            system_status=4,          #MAV_STATE_ACTIVE
        )

    def _on_attitude_target(self, msg: AttitudeTarget):
        #Only forward once recv_match has populated the autopilot's IDs from
        #its heartbeat; before that target_system is 0 and the send is a no-op.
        if not getattr(self.conn, "target_system", 0):
            return

        #PX4 ignores setpoints unless the vehicle is in offboard mode and
        #armed. Do that once, the first time a real setpoint arrives.
        if self.auto_offboard_arm and not self._offboard_armed:
            self._enter_offboard_and_arm()
            self._offboard_armed = True

        #time_boot_ms is a uint32 in SET_ATTITUDE_TARGET. Clock skew under
        #WSL can make (now - boot) momentarily negative, which overflows the
        #unsigned pack and raises struct.error. Clamp to [0, 2**32) so a bad
        #clock reading can never crash the bridge.
        now_ms = int(time.time() * 1000) - self._boot_ms
        now_ms = max(0, now_ms) & 0xFFFFFFFF
        self.conn.mav.set_attitude_target_send(
            now_ms,
            self.conn.target_system,
            self.conn.target_component,
            int(msg.type_mask) & 0xFF,
            [
                float(msg.orientation.w),
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
            ],
            float(msg.body_rate.x),   # roll rate
            float(msg.body_rate.y),   # pitch rate
            float(msg.body_rate.z),   # yaw rate
            float(msg.thrust),
            )
        self.setpoints_forwarded += 1

    def _enter_offboard_and_arm(self):
        #PX4 offboard is custom main mode 6. Set it, then arm.
        self.get_logger().info("Entering offboard mode and arming...")
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6,  # PX4 offboard main mode
            0, 0, 0, 0, 0,
        )
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0,
        )

    def _log_stats(self):
        if self.imu_received == 0:
            self.get_logger().warn(
                f"No HIGHRES_IMU received yet. "
                f"heartbeats={self.heartbeats_received} unknown={self.unknown_messages}"
            )
            return
        sim_time_s = (self.last_sim_time_us or 0) / 1_000_000.0
        self.get_logger().info(
            f"imu_published={self.imu_published} "
            f"heartbeats={self.heartbeats_received} "
            f"attitude={self.attitude_received} "
            f"setpoints_fwd={self.setpoints_forwarded} "
            f"unknown={self.unknown_messages} "
            f"sim_time={sim_time_s:.2f}s"
        )

    def _handle_local_position(self, msg):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "sim_ned"
        pose.pose.position.x = float(msg.x)
        pose.pose.position.y = float(msg.y)
        pose.pose.position.z = float(msg.z)
        self.pose_pub.publish(pose)

    def _handle_odometry(self, msg):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "sim_ned"
        odom.child_frame_id = "sim_body"

        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.position.z = float(msg.z)

        # Sim sends quaternion as [w, x, y, z]
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

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = MavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()