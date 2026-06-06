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
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from pymavlink import mavutil


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

        conn_str = self.get_parameter("connection_string").value
        topic = self.get_parameter("imu_topic").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        heartbeat_period = self.get_parameter("heartbeat_period_s").value

        self.get_logger().info(f"Opening MAVLink connection: {conn_str}")
        self.conn = mavutil.mavlink_connection(
            conn_str,
            source_system=MAVLINK_SYSTEM_ID,
            source_component=MAVLINK_COMPONENT_ID,
            dialect="common",
        )

        self.imu_pub = self.create_publisher(Imu, topic, 50)

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
            else:
                self.unknown_messages += 1

    def _handle_highres_imu(self, msg):
        self.imu_received += 1
        self.last_sim_time_us = msg.time_usec

        imu = Imu()

        #Stamp from sim time (time_usec, microseconds)
        #Any drift between sim time and our wall clock would ruin the
        #camera-IMU offset OpenVINS spends real effort calibrating.
        sec = msg.time_usec // 1_000_000
        nsec = (msg.time_usec % 1_000_000) * 1_000
        imu.header.stamp.sec = int(sec)
        imu.header.stamp.nanosec = int(nsec)
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
            f"unknown={self.unknown_messages} "
            f"sim_time={sim_time_s:.2f}s"
        )


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