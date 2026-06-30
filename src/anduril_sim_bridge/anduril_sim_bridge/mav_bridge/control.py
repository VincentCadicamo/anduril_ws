"""
control.py: outbound control path - offboard/arm sequencing and setpoint
forwarding. Owns the arm state machine; borrows the transport to send.

Separated from transport because "should I arm, and have I armed yet" is
policy, not plumbing. The transport stays a dumb pipe; this decides when to
push mode/arm commands and how to pack a setpoint.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from pymavlink import mavutil

from .transport import MavlinkTransport


_PX4_OFFBOARD_MAIN_MODE = 6
_MAVLINK_CMD_SIM_RESET = 31000

_VELOCITY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


class OffboardController:
    def __init__(
            self,
            transport: MavlinkTransport,
            auto_offboard_arm: bool,
            log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._tx = transport
        self._auto = auto_offboard_arm
        self._log = log or (lambda _m: None)
        self._armed = False
        self._boot_ms = int(time.time() * 1000)

    def forward_attitude_target(self, msg) -> bool:
        """
        Forward a mavros_msgs/AttitudeTarget to the sim. Returns True if sent.
        No-op (returns False) until the peer's ids are known from its heartbeat.
        """
        if not self._tx.target_system:
            return False

        if self._auto and not self._armed:
            self._enter_offboard_and_arm()
            self._armed = True

        # time_boot_ms is uint32; clock skew under WSL can make (now - boot)
        # momentarily negative, overflowing the unsigned pack. Clamp to range.
        now_ms = int(time.time() * 1000) - self._boot_ms
        now_ms = max(0, now_ms) & 0xFFFFFFFF

        self._tx.conn.mav.set_attitude_target_send(
            now_ms,
            self._tx.target_system,
            self._tx.target_component,
            int(msg.type_mask) & 0xFF,
            [
                float(msg.orientation.w),
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
            ],
            float(msg.body_rate.x),
            float(msg.body_rate.y),
            float(msg.body_rate.z),
            float(msg.thrust),
            )
        return True

    def arm(self) -> None:
        """Arm the vehicle unconditionally (no mode change)."""
        self._tx.conn.mav.command_long_send(
            self._tx.target_system,
            self._tx.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0,
        )

    def send_velocity_ned(self, vx: float, vy: float, vz: float) -> bool:
        """NED velocity setpoint (m/s). Returns False if peer not yet seen."""
        if not self._tx.target_system:
            return False
        now_ms = int(time.time() * 1000) - self._boot_ms
        now_ms = max(0, now_ms) & 0xFFFFFFFF
        self._tx.conn.mav.set_position_target_local_ned_send(
            now_ms,
            self._tx.target_system,
            self._tx.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _VELOCITY_MASK,
            0.0, 0.0, 0.0,    # ignored position
            vx, vy, vz,        # velocity NED m/s
            0.0, 0.0, 0.0,    # ignored acceleration
            0.0,               # ignored yaw
            0.0,               # ignored yaw rate
        )
        return True

    def send_motor_rpms(self, rpms: list) -> bool:
        """Direct motor RPM control. rpms is a list of up to 4 floats [FL, FR, BL, BR]."""
        if not self._tx.target_system:
            return False
        padded = list(rpms) + [0.0] * (8 - len(rpms))
        self._tx.conn.mav.set_actuator_control_target_send(
            int(time.time() * 1_000_000),
            self._tx.target_system,
            self._tx.target_component,
            0,           # group_mlx
            padded[:8],
        )
        return True

    def send_sim_reset(self) -> None:
        """Send the competition's custom SIM_RESET command (id 31000)."""
        self._tx.conn.mav.command_long_send(
            self._tx.target_system,
            self._tx.target_component,
            _MAVLINK_CMD_SIM_RESET,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )

    def _enter_offboard_and_arm(self) -> None:
        self._log("Entering offboard mode and arming...")
        self._tx.conn.mav.command_long_send(
            self._tx.target_system,
            self._tx.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            _PX4_OFFBOARD_MAIN_MODE,
            0, 0, 0, 0, 0,
        )
        self._tx.conn.mav.command_long_send(
            self._tx.target_system,
            self._tx.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0,
        )