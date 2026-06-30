"""
TimeAligner: single source of truth for putting every sensor stream onto one
shared timebase, so OpenVINS can correlate camera and IMU samples.

Design
------
The camera bridge stamps images with the simulator's raw `sim_time_ns` field
from the UDP header, which the sim fills with the **Unix epoch** (wall-clock)
time in nanoseconds.

The MAVLink HIGHRES_IMU.time_usec field is **sim boot time** (nanoseconds
since simulation start, here ~27 seconds at first sample vs ~1.78e18 for the
camera). These are completely different clock domains; directly stamping IMU
messages with raw sim time causes OpenVINS's camera-before-IMU gate to never
open (camera.timestamp >> imu.timestamp forever).

Fix: on the first IMU sample, capture the epoch offset:
    epoch_offset_ns = wall_clock_ns_now - imu_sim_ns
and add it to every subsequent IMU stamp so IMU timestamps land on Unix epoch
time, matching the camera.

The per-stream manual offset (set_offset) is additive on top of the epoch
offset and is only needed if individual streams need fine-grained correction
after the epoch is already aligned.
"""

from __future__ import annotations

import time as _time_module
from dataclasses import dataclass
from typing import Callable, Optional


NS_PER_SEC = 1_000_000_000


@dataclass
class _StreamObservation:
    first_sim_ns: int
    count: int = 0


class TimeAligner:
    """
    Converts a stream's native sim timestamp into ROS (sec, nanosec) on a
    single shared timebase anchored to Unix epoch.

    On the very first call, the aligner captures
        epoch_offset_ns = wall_clock_ns - sim_time_ns
    and applies it to every subsequent stamp so that all streams end up on
    Unix epoch time — the same timebase the camera bridge uses.

    Parameters
    ----------
    log : optional callable(str)
        Sink for diagnostic messages. If None, diagnostics are silently
        dropped — the conversion still works.
    warn_threshold_ns : int
        If two streams' first stamps differ by more than this (after the epoch
        offset is applied), warn that their origins may not match.
        Default 100 ms.
    """

    def __init__(
            self,
            log: Optional[Callable[[str], None]] = None,
            warn_threshold_ns: int = 100 * 1_000_000,
    ) -> None:
        self._log = log
        self._warn_threshold_ns = warn_threshold_ns
        self._streams: dict[str, _StreamObservation] = {}
        self._offsets_ns: dict[str, int] = {}
        self._cross_checked = False
        self._epoch_offset_ns: Optional[int] = None

    # ------------------------------------------------------------------ #
    # Conversion
    # ------------------------------------------------------------------ #
    def stamp_from_us(self, sim_time_us: int, stream: str) -> tuple[int, int]:
        """Convert a microsecond sim stamp (e.g. HIGHRES_IMU.time_usec)."""
        return self.stamp_from_ns(sim_time_us * 1000, stream)

    def stamp_from_ns(self, sim_time_ns: int, stream: str) -> tuple[int, int]:
        """Convert a nanosecond sim stamp (e.g. HIGHRES_IMU-derived)."""
        if self._epoch_offset_ns is None:
            wall_ns = _time_module.time_ns()
            self._epoch_offset_ns = wall_ns - sim_time_ns
            self._info(
                f"[TimeAligner] epoch offset captured: "
                f"wall={wall_ns} ns, sim={sim_time_ns} ns, "
                f"offset={self._epoch_offset_ns} ns "
                f"(sim was {sim_time_ns / NS_PER_SEC:.3f} s since boot)"
            )
        self._observe(stream, sim_time_ns)
        corrected = (sim_time_ns
                     + self._epoch_offset_ns
                     + self._offsets_ns.get(stream, 0))
        return (corrected // NS_PER_SEC, corrected % NS_PER_SEC)

    # ------------------------------------------------------------------ #
    # Origin cross-check (fires after epoch offset is established)
    # ------------------------------------------------------------------ #
    def _observe(self, stream: str, sim_time_ns: int) -> None:
        obs = self._streams.get(stream)
        if obs is None:
            obs = _StreamObservation(first_sim_ns=sim_time_ns)
            self._streams[stream] = obs
            self._maybe_cross_check()
        obs.count += 1

    def _maybe_cross_check(self) -> None:
        if self._cross_checked or len(self._streams) < 2:
            return
        self._cross_checked = True
        items = sorted(self._streams.items(), key=lambda kv: kv[1].first_sim_ns)
        lo_name, lo = items[0]
        hi_name, hi = items[-1]
        delta_ns = hi.first_sim_ns - lo.first_sim_ns
        msg = (
            f"[TimeAligner] first-stamp cross-check: "
            f"{hi_name}={hi.first_sim_ns} ns, {lo_name}={lo.first_sim_ns} ns, "
            f"delta={delta_ns / 1e6:.3f} ms"
        )
        if delta_ns > self._warn_threshold_ns:
            msg += (
                " -- LARGE: streams may not share a clock origin. If OpenVINS "
                "fails to initialize, set_offset() on one stream to correct it."
            )
            self._warn(msg)
        else:
            msg += " -- streams appear to share a clock origin."
            self._info(msg)

    def set_offset(self, stream: str, offset_ns: int) -> None:
        """
        Apply a constant additive correction (nanoseconds) to one stream,
        on top of the epoch offset. Only needed for fine-grained per-stream
        skew after the epoch is already aligned.
        """
        self._offsets_ns[stream] = offset_ns
        self._info(f"[TimeAligner] offset for '{stream}' set to {offset_ns} ns")

    # ------------------------------------------------------------------ #
    def _info(self, m: str) -> None:
        if self._log:
            self._log(m)

    def _warn(self, m: str) -> None:
        if self._log:
            self._log(m)
