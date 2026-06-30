"""
transport.py: the single MAVLink connection owner. Knows nothing about ROS.

Responsibilities:
  - own the pymavlink connection
  - pump the receive queue and dispatch to registered handlers by msg type
  - emit our own HEARTBEAT (>=2 Hz per spec 4.4)
  - answer TIMESYNC probes
  - track the peer (autopilot) system/component ids as they appear

Everything ROS-specific (publishers, message conversion) lives in the node,
which registers handlers here. This keeps transport unit-testable against a
fake connection and lets the node stay thin.

The transport does NOT block in its constructor. The old
`_wait_for_first_heartbeat` busy-loop is replaced by a non-blocking
`peer_seen` flag the node can poll or react to via the HEARTBEAT handler.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from pymavlink import mavutil


MAVLINK_COMPONENT_ID = 191   # MAV_COMP_ID_ONBOARD_COMPUTER
MAVLINK_SYSTEM_ID = 245      # must not collide with the sim's

# Heartbeat identity: we are a companion computer, not an autopilot.
_MAV_TYPE_ONBOARD_CONTROLLER = 18
_MAV_AUTOPILOT_INVALID = 8
_MAV_STATE_ACTIVE = 4

HandlerFn = Callable[[object], None]


class MavlinkTransport:
    def __init__(
            self,
            connection_string: str,
            log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._log = log or (lambda _m: None)
        self._log(f"Opening MAVLink connection: {connection_string}")
        self.conn = mavutil.mavlink_connection(
            connection_string,
            source_system=MAVLINK_SYSTEM_ID,
            source_component=MAVLINK_COMPONENT_ID,
            dialect="common",
        )
        self._handlers: dict[str, HandlerFn] = {}
        self.peer_seen = False
        self.heartbeats_received = 0
        self.unknown_messages = 0

        # Internal: always answer timesync + track heartbeats, regardless of
        # what the node registers.
        self.register("TIMESYNC", self._handle_timesync)

    # ------------------------------------------------------------------ #
    def register(self, msg_type: str, fn: HandlerFn) -> None:
        """Register a handler for a MAVLink message type. One per type."""
        self._handlers[msg_type] = fn

    @property
    def target_system(self) -> int:
        return getattr(self.conn, "target_system", 0) or 0

    @property
    def target_component(self) -> int:
        return getattr(self.conn, "target_component", 0) or 0

    # ------------------------------------------------------------------ #
    def poll(self, max_messages: int = 64) -> None:
        """Drain up to max_messages from the receive queue (non-blocking)."""
        for _ in range(max_messages):
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                self.heartbeats_received += 1
                if not self.peer_seen:
                    self.peer_seen = True
                    self._log(
                        f"Peer connected (system_id={msg.get_srcSystem()}, "
                        f"component_id={msg.get_srcComponent()})"
                    )
            handler = self._handlers.get(mtype)
            if handler is not None:
                handler(msg)
            elif mtype not in ("HEARTBEAT",):
                self.unknown_messages += 1

    def send_heartbeat(self) -> None:
        self.conn.mav.heartbeat_send(
            type=_MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot=_MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=_MAV_STATE_ACTIVE,
        )

    def send_timesync_request(self) -> None:
        """Actively probe the sim for clock synchronisation (10 Hz recommended)."""
        self.conn.mav.timesync_send(tc1=time.time_ns(), ts1=0)

    def _handle_timesync(self, msg) -> None:
        # tc1 == 0 means the sender wants us to fill our timestamp and echo.
        if msg.tc1 == 0:
            self.conn.mav.timesync_send(tc1=time.time_ns(), ts1=msg.ts1)

    def close(self) -> None:
        self.conn.close()