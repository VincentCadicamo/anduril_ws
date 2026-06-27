"""
Manual keyboard teleoperation node.

Reads held keys via pynput and publishes attitude-rate + thrust setpoints
as mavros_msgs/AttitudeTarget on /setpoint/attitude_target at CONTROL_HZ.

This node does NOT open a MAVLink connection. A downstream bridge
(e.g. mav_bridge) subscribes to /setpoint/attitude_target and forwards
the values to the simulator via set_attitude_target_send. This keeps a
single MAVLink owner and avoids two processes fighting over UDP 14550.

Key map (matches the printed help):
  W / S    pitch forward / backward
  A / D    strafe left / right   (roll)
  Q / E    rotate left / right   (yaw)
  space    ascend  (increase collective thrust)
  shift    descend (decrease collective thrust)
  p        quit
  (release all keys -> zero rates, thrust held -> hover)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.msg import AttitudeTarget

from pynput import keyboard


# AttitudeTarget.type_mask: ignore the orientation quaternion, use body rates.
# Bit 7 (value 128) = IGNORE_ATTITUDE in mavros_msgs/AttitudeTarget.
IGNORE_ATTITUDE = AttitudeTarget.IGNORE_ATTITUDE


class KeyboardInput:
    """Thread-backed held-key tracker using a pynput listener."""

    def __init__(self):
        self.held_keys = set()
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

    def _on_press(self, key):
        try:
            self.held_keys.add(key.char.lower())   # regular keys
        except AttributeError:
            self.held_keys.add(key)                # special keys

    def _on_release(self, key):
        try:
            self.held_keys.discard(key.char.lower())
        except AttributeError:
            self.held_keys.discard(key)

    def is_held(self, ch):
        return ch in self.held_keys

    def stop(self):
        self._listener.stop()


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control")

        # --- parameters ---
        self.declare_parameter("control_hz", 250.0)
        self.declare_parameter("pitch_rate", 0.4)   # rad/s
        self.declare_parameter("roll_rate", 0.4)    # rad/s
        self.declare_parameter("yaw_rate", 0.4)     # rad/s
        self.declare_parameter("base_thrust", 0.0)  # 0.0 - 1.0; start grounded
        # Thrust change per SECOND while space/shift is held (loop-rate
        # independent). At 0.25, holding for 1 s moves thrust by 0.25.
        self.declare_parameter("thrust_rate", 0.25)
        self.declare_parameter("setpoint_topic", "/setpoint/attitude_target")

        self.control_hz = self.get_parameter("control_hz").value
        self.pitch_rate = self.get_parameter("pitch_rate").value
        self.roll_rate = self.get_parameter("roll_rate").value
        self.yaw_rate = self.get_parameter("yaw_rate").value
        self.thrust_rate = self.get_parameter("thrust_rate").value
        self._thrust = self.get_parameter("base_thrust").value
        topic = self.get_parameter("setpoint_topic").value

        # Flight is gated: nothing is commanded until the operator presses the
        # arm key ('r'). Until then we publish a zero-thrust, zero-rate hold so
        # the stream exists (PX4/offboard needs a steady setpoint rate) but the
        # drone does not move. This stops the auto-climb on sim start and lets
        # OpenVINS initialize from a controlled, deliberate motion.
        self._armed = False

        # Per-tick time step, used to make thrust change rate-based.
        self._dt = 1.0 / self.control_hz

        # Setpoints are a stream where only the latest matters; use a small
        # best-effort queue so a slow subscriber can't back-pressure teleop.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(AttitudeTarget, topic, qos)

        self.keyboard = KeyboardInput()
        self._print_help()

        period = 1.0 / self.control_hz
        self.create_timer(period, self._tick)

    def _print_help(self):
        print("\n" + "=" * 44)
        print("  DRONE KEYBOARD CONTROLLER")
        print("=" * 44)
        print("  R --------- ARM (controls go live)")
        print("  W --------- fly forward    (pitch)")
        print("  S --------- fly backward   (pitch)")
        print("  A --------- strafe left    (roll)")
        print("  D --------- strafe right   (roll)")
        print("  Q --------- rotate left    (yaw)")
        print("  E --------- rotate right   (yaw)")
        print("  'space' --- ascend         (up)")
        print("  'shift' --- descend        (down)")
        print("  p ------------------------- quit")
        print("  (press R first; release keys to hover)")
        print("=" * 44 + "\n")

    def _tick(self):
        kb = self.keyboard

        if kb.is_held('p'):
            self.get_logger().info("QUITTING")
            self.keyboard.stop()
            rclpy.shutdown()
            return

        # Arm gate. Until 'r' is pressed, publish a zero hold so the setpoint
        # stream exists but the drone stays put. This prevents the auto-climb
        # at sim start and gives OpenVINS a stable platform before motion.
        if not self._armed:
            if kb.is_held('r'):
                self._armed = True
                self.get_logger().info("ARMED - controls live")
            else:
                self._publish(0.0, 0.0, 0.0, 0.0)
                return

        pitch_rate = 0.0
        roll_rate = 0.0
        yaw_rate = 0.0

        if kb.is_held('w'):
            pitch_rate = -self.pitch_rate    # forward (nose down)
        if kb.is_held('s'):
            pitch_rate = self.pitch_rate     # backward
        if kb.is_held('a'):
            roll_rate = -self.roll_rate      # strafe left
        if kb.is_held('d'):
            roll_rate = self.roll_rate       # strafe right
        if kb.is_held('q'):
            yaw_rate = -self.yaw_rate        # rotate left
        if kb.is_held('e'):
            yaw_rate = self.yaw_rate         # rotate right

        # More collective thrust = up in NED. Rate-based: thrust changes by
        # thrust_rate per second held, independent of loop frequency.
        if kb.is_held(keyboard.Key.space):
            self._thrust = min(self._thrust + self.thrust_rate * self._dt, 1.0)
        if kb.is_held(keyboard.Key.shift):
            self._thrust = max(self._thrust - self.thrust_rate * self._dt, 0.0)

        self._publish(roll_rate, pitch_rate, yaw_rate, self._thrust)

    def _publish(self, roll_rate, pitch_rate, yaw_rate, thrust):
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.type_mask = IGNORE_ATTITUDE
        # Orientation quaternion is ignored per type_mask; send identity.
        msg.orientation.w = 1.0
        msg.body_rate.x = float(roll_rate)
        msg.body_rate.y = float(pitch_rate)
        msg.body_rate.z = float(yaw_rate)
        msg.thrust = float(thrust)
        self.pub.publish(msg)

    def destroy_node(self):
        self.keyboard.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()