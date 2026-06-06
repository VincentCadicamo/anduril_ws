#!/usr/bin/env python3
"""
Fake-sim MAVLink sender. Emits HEARTBEAT and HIGHRES_IMU over UDP, and
sends periodic TIMESYNC probes so you can verify the bridge replies.
Mimics the AI Grand Prix simulator's MAVLink output.

Useful for testing anduril_sim_bridge.mav_bridge before the real sim is
available. Synthesizes IMU data so you can sanity-check what comes out of /imu0.

Usage:
  python3 mav_sim.py                    # localhost, default rates
  python3 mav_sim.py --host 10.0.0.42   # remote bridge
  python3 mav_sim.py --imu-hz 100       # override IMU rate
"""

import argparse
import math
import time

from pymavlink import mavutil


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=14540)
    p.add_argument("--imu-hz", type=float, default=200.0)
    p.add_argument("--heartbeat-hz", type=float, default=2.0)
    p.add_argument("--timesync-hz", type=float, default=1.0)
    p.add_argument("--system-id", type=int, default=1,
                   help="Sim's MAVLink system id (1 is conventional)")
    args = p.parse_args()

    # udpout: we initiate the send toward the bridge's listening socket.
    conn_str = f"udpout:{args.host}:{args.port}"
    print(f"Sending MAVLink to {conn_str}")
    conn = mavutil.mavlink_connection(
        conn_str,
        source_system=args.system_id,
        source_component=1,
        dialect="common",
    )

    imu_period = 1.0 / args.imu_hz
    heartbeat_period = 1.0 / args.heartbeat_hz
    timesync_period = 1.0 / args.timesync_hz

    start_wall = time.monotonic()
    last_imu = 0.0
    last_heartbeat = 0.0
    last_timesync = 0.0
    sim_t0_us = time.time_ns() // 1000

    imu_count = 0
    hb_count = 0
    timesync_sent = 0
    timesync_replies = 0

    print("Sending. Ctrl-C to stop.")
    try:
        while True:
            now = time.monotonic()
            sim_now_us = sim_t0_us + int((now - start_wall) * 1_000_000)

            #HEARTBEAT: presents as a quadrotor running PX4.
            if now - last_heartbeat >= heartbeat_period:
                conn.mav.heartbeat_send(
                    type=2,            # MAV_TYPE_QUADROTOR
                    autopilot=12,      # MAV_AUTOPILOT_PX4
                    base_mode=0,
                    custom_mode=0,
                    system_status=4,   # MAV_STATE_ACTIVE
                )
                hb_count += 1
                last_heartbeat = now


            if now - last_imu >= imu_period:
                t = now - start_wall
                wx = 0.05 * math.sin(2 * math.pi * 0.3 * t)
                wy = 0.05 * math.cos(2 * math.pi * 0.3 * t)
                wz = 0.0
                ax = 0.0
                ay = 0.0
                az = -9.81   # NED body frame gravity reads -g on z
                conn.mav.highres_imu_send(
                    time_usec=sim_now_us,
                    xacc=ax, yacc=ay, zacc=az,
                    xgyro=wx, ygyro=wy, zgyro=wz,
                    xmag=0.0, ymag=0.0, zmag=0.0,
                    abs_pressure=1013.25,
                    diff_pressure=0.0,
                    pressure_alt=0.0,
                    temperature=20.0,
                    fields_updated=0x3F,
                )
                imu_count += 1
                last_imu = now

            if now - last_timesync >= timesync_period:
                conn.mav.timesync_send(tc1=0, ts1=time.time_ns())
                timesync_sent += 1
                last_timesync = now

            for _ in range(8):
                msg = conn.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == "TIMESYNC" and msg.tc1 != 0:
                    timesync_replies += 1

            if imu_count and imu_count % 1000 == 0:
                print(
                    f"sent imu={imu_count} hb={hb_count} "
                    f"timesync_sent={timesync_sent} "
                    f"timesync_replies={timesync_replies}"
                )

            time.sleep(min(imu_period, heartbeat_period) / 4)

    except KeyboardInterrupt:
        print(
            f"\nstopped. imu={imu_count} hb={hb_count} "
            f"timesync_sent={timesync_sent} timesync_replies={timesync_replies}"
        )


if __name__ == "__main__":
    main()
