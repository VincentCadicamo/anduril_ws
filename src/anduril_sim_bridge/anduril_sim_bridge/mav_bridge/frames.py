"""
frames.py: every axis/frame convention decision lives here, so changing a
convention is a single edit rather than a hunt across converters.

Current assumption (UNVERIFIED - see note)
-------------------------------------------
MAVLink HIGHRES_IMU reports body-frame angular rate and linear acceleration in
FRD (forward-right-down), matching MAVLink BODY_NED. The previous code assumed
body-to-IMU is identity and copied axes straight through.

That assumption has NOT been empirically confirmed for this simulator. The
raw-IMU debug dump (first N samples, at rest) is the instrument to confirm it:
  - |accel| should be ~9.81 m/s^2 (one axis carries gravity). If ~1.0, the sim
    sends g, not m/s^2 -> set ACCEL_IS_IN_G = True.
  - at rest, +Z (down) in FRD should read about +9.81 (gravity reaction up is
    measured as specific force pointing up, i.e. -accel... confirm sign here).
  - gyro should be ~0 on all axes at rest.

Keeping the mapping isolated means: when the dump confirms (or refutes) the
convention, you flip a flag or swap one function body, and every consumer
inherits the fix.
"""

from __future__ import annotations

from dataclasses import dataclass


# Set True if the at-rest |accel| reads ~1.0 instead of ~9.81 (sim sends g).
ACCEL_IS_IN_G = False
_G = 9.80665


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float


def imu_gyro_body(xgyro: float, ygyro: float, zgyro: float) -> Vec3:
    """
    Map raw HIGHRES_IMU gyro axes to the body frame published downstream.
    Identity under the FRD assumption. Edit ONLY here to change convention.
    """
    return Vec3(float(xgyro), float(ygyro), float(zgyro))


def imu_accel_body(xacc: float, yacc: float, zacc: float) -> Vec3:
    """
    Map raw HIGHRES_IMU accel axes to the body frame published downstream.
    Identity under the FRD assumption, with optional g->m/s^2 scaling.
    Edit ONLY here to change convention.
    """
    scale = _G if ACCEL_IS_IN_G else 1.0
    return Vec3(float(xacc) * scale, float(yacc) * scale, float(zacc) * scale)


def accel_magnitude(v: Vec3) -> float:
    return (v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5