"""
ekf.py — Error-State EKF for drone localization. Pure numpy, no ROS.

Fuses IMU (propagation) with PnP gate position measurements (update).

Coordinate frames
-----------------
World  : NED (North-East-Down), fixed inertial frame.
Body   : FRD (Forward-Right-Down).  q represents body→world rotation.
cam0   : Published by gate_pose_node in FLU (Forward-Left-Up).

State
-----
Nominal  x  = [p(3)  v(3)  q(4)  bg(3)  ba(3)]   (16 elements)
Error   δx  = [δp(3) δv(3) δθ(3) δbg(3) δba(3)]  (15 elements)

Quaternion convention: [w, x, y, z]  (ROS standard).
"""

from __future__ import annotations

import numpy as np

# Gravity vector in world NED frame (down = +Z).
_G_NED = np.array([0.0, 0.0, 9.80665])

# Rotation: cam0 FLU → body FRD
# Derived from T_imu_cam (kalibr, 20° upward tilt) composed with the
# FLU→optical axis swap applied in gate_pose_node.py:
#   position.x = z_c  (optical Z → FLU X)
#   position.y = -x_c (optical X → FLU Y)
#   position.z = -y_c (optical Y → FLU Z)
# So: R_FLU_to_optical = [[0,-1,0],[0,0,-1],[1,0,0]]
# And: R_imu_cam (optical→body) from kalibr =
#   [[0, 0.34202, 0.939693],
#    [1, 0,       0       ],
#    [0, 0.939693,-0.34202]]
# _R_CAM_FLU_TO_BODY = R_imu_cam @ R_FLU_to_optical
_R_CAM_FLU_TO_BODY: np.ndarray = np.array(
    [
        [ 0.939693,  0.0, -0.34202 ],
        [ 0.0,      -1.0,  0.0     ],
        [-0.34202,   0.0, -0.939693],
    ],
    dtype=np.float64,
)


# ---------------------------------------------------------------------------
# Quaternion / rotation helpers (all pure numpy)
# ---------------------------------------------------------------------------

def _skew(v: np.ndarray) -> np.ndarray:
    """3-vector → 3×3 skew-symmetric matrix: skew(v) @ u == v × u."""
    return np.array(
        [[ 0.0,  -v[2],  v[1]],
         [ v[2],  0.0,  -v[0]],
         [-v[1],  v[0],  0.0 ]],
        dtype=np.float64,
    )


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Quaternion multiply [w,x,y,z] × [w,x,y,z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], dtype=np.float64)


def _qnorm(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion."""
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def _qrot(q: np.ndarray) -> np.ndarray:
    """Quaternion [w,x,y,z] → 3×3 rotation matrix (body→world)."""
    w, x, y, z = q
    return np.array([
        [1.0 - 2.0*(y*y + z*z),       2.0*(x*y - w*z),       2.0*(x*z + w*y)],
        [      2.0*(x*y + w*z),  1.0 - 2.0*(x*x + z*z),       2.0*(y*z - w*x)],
        [      2.0*(x*z - w*y),        2.0*(y*z + w*x),  1.0 - 2.0*(x*x + y*y)],
    ], dtype=np.float64)


def _rotvec_to_quat(v: np.ndarray) -> np.ndarray:
    """Small rotation vector → quaternion [w,x,y,z]."""
    angle = np.linalg.norm(v)
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = v / angle
    s = np.sin(angle * 0.5)
    return np.array([np.cos(angle * 0.5), s*axis[0], s*axis[1], s*axis[2]])


# ---------------------------------------------------------------------------
# EKF
# ---------------------------------------------------------------------------

class DroneEKF:
    """
    Error-State EKF fusing IMU propagation with PnP gate position updates.

    Typical use
    -----------
    ekf = DroneEKF()
    ekf.initialize(p_world_ned, q_body_to_world)

    # At each IMU sample (~120 Hz):
    ekf.propagate(omega_m, f_m, dt)

    # At each gate detection (~30 Hz):
    accepted, innov, mahal = ekf.update_gate(z_flu, R_meas_flu, gate_pos_world)
    """

    N: int = 15  # error-state dimension

    def __init__(
        self,
        sigma_acc: float   = 0.3,     # m/s²/√Hz  — accelerometer noise
        sigma_gyr: float   = 0.02,    # rad/s/√Hz — gyroscope noise
        sigma_ba_rw: float = 0.002,   # m/s²·√Hz  — accel bias random walk
        sigma_bg_rw: float = 2e-4,    # rad/s·√Hz — gyro bias random walk
        chi2_thresh: float = 11.345,  # 3-DOF 99 % gate
    ) -> None:
        self._sa2  = sigma_acc   ** 2
        self._sg2  = sigma_gyr   ** 2
        self._sba2 = sigma_ba_rw ** 2
        self._sbg2 = sigma_bg_rw ** 2
        self.chi2_thresh = chi2_thresh

        # Nominal state
        self.p:  np.ndarray = np.zeros(3)
        self.v:  np.ndarray = np.zeros(3)
        self.q:  np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])
        self.bg: np.ndarray = np.zeros(3)
        self.ba: np.ndarray = np.zeros(3)

        # Error covariance — generous initial uncertainty
        _p0 = np.array([
            1.0,  1.0,  1.0,     # position  ±1 m
            1.0,  1.0,  1.0,     # velocity  ±1 m/s
            0.3,  0.3,  0.3,     # attitude  ±~17°
            0.03, 0.03, 0.03,    # gyro bias
            0.1,  0.1,  0.1,     # accel bias
        ])
        self.P: np.ndarray = np.diag(_p0 ** 2)

        self.initialized: bool = False

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def initialize(
        self,
        p_world: np.ndarray,
        q_world: np.ndarray | None = None,
    ) -> None:
        """Seed the filter with a known world-NED position (and optional attitude)."""
        self.p[:] = p_world
        self.v[:] = 0.0
        self.q[:] = _qnorm(q_world) if q_world is not None else np.array([1.0, 0.0, 0.0, 0.0])
        self.bg[:] = 0.0
        self.ba[:] = 0.0
        self.initialized = True

    # ------------------------------------------------------------------
    # IMU propagation
    # ------------------------------------------------------------------

    def propagate(
        self,
        omega_m: np.ndarray,   # measured angular rate (rad/s), body FRD
        f_m:     np.ndarray,   # measured specific force (m/s²), body FRD
        dt:      float,
    ) -> None:
        """Propagate nominal state and error covariance with one IMU sample."""
        if not self.initialized or dt <= 0.0 or dt > 0.5:
            return

        # Bias-corrected measurements
        omega = omega_m - self.bg
        f     = f_m     - self.ba

        R = _qrot(self.q)   # body→world

        # --- Nominal state propagation (midpoint Euler) ----------------------
        a_world = R @ f + _G_NED
        p_new   = self.p + self.v * dt + 0.5 * a_world * (dt * dt)
        v_new   = self.v + a_world * dt
        q_new   = _qnorm(_qmul(self.q, _rotvec_to_quat(omega * dt)))

        # --- Error-state transition matrix F (15×15) -------------------------
        F = np.eye(self.N)
        F[0:3,  3:6]   = np.eye(3) * dt          # δp ← δv · dt
        F[3:6,  6:9]   = -R @ _skew(f) * dt       # δv ← −R skew(f) δθ dt
        F[3:6,  12:15] = R * dt                    # δv ← R δba dt
        F[6:9,  6:9]  -= _skew(omega) * dt         # δθ ← −skew(ω) δθ dt
        F[6:9,  9:12]  = -np.eye(3) * dt           # δθ ← −δbg dt

        # --- Discrete process noise Q (additive) -----------------------------
        Q = np.zeros((self.N, self.N))
        Q[3:6,   3:6]   = np.eye(3) * (self._sa2  * dt)
        Q[6:9,   6:9]   = np.eye(3) * (self._sg2  * dt)
        Q[9:12,  9:12]  = np.eye(3) * (self._sbg2 * dt)
        Q[12:15, 12:15] = np.eye(3) * (self._sba2 * dt)

        # --- Covariance propagation ------------------------------------------
        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)   # symmetrize against fp drift

        # Commit nominal state
        self.p = p_new
        self.v = v_new
        self.q = q_new

    # ------------------------------------------------------------------
    # Gate position measurement update
    # ------------------------------------------------------------------

    def update_zupt(self, sigma_v: float = 0.02) -> None:
        """
        Zero-Velocity UPdaTe: inject v = 0 when the drone is stationary.

        Measurement model: z = 0₃ = v_world + noise
        Jacobian H = [0 | I | 0 | 0 | 0]  (only velocity rows are non-zero)

        This anchors velocity drift and lets the filter converge on accelerometer
        bias before takeoff. Call whenever stationarity is detected in the node.
        """
        if not self.initialized:
            return

        H = np.zeros((3, self.N))
        H[:, 3:6] = np.eye(3)

        R_zupt = np.eye(3) * (sigma_v ** 2)
        innov  = -self.v                          # z − h(x) = 0 − v

        S = H @ self.P @ H.T + R_zupt
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        K  = self.P @ H.T @ S_inv
        dx = K @ innov

        self.p  += dx[0:3]
        self.v  += dx[3:6]
        self.q   = _qnorm(_qmul(self.q, _rotvec_to_quat(dx[6:9])))
        self.bg += dx[9:12]
        self.ba += dx[12:15]

        I_KH   = np.eye(self.N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_zupt @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def update_gate(
        self,
        z_flu:           np.ndarray,   # gate position in cam0 FLU frame (3,)
        R_meas_flu:      np.ndarray,   # 3×3 measurement covariance in cam0 FLU
        gate_pos_world:  np.ndarray,   # gate centre in world NED (3,)
    ) -> tuple[bool, np.ndarray, float]:
        """
        Update state with one gate position measurement.

        Returns
        -------
        accepted : bool  — False if chi-squared gate rejected the update
        innov    : (3,)  — innovation in body frame
        mahal    : float — Mahalanobis distance²
        """
        if not self.initialized:
            return False, np.zeros(3), 0.0

        # Convert measurement and its covariance to body FRD frame
        z_body      = _R_CAM_FLU_TO_BODY @ z_flu
        R_meas_body = _R_CAM_FLU_TO_BODY @ R_meas_flu @ _R_CAM_FLU_TO_BODY.T

        # Predicted gate position in body frame
        R_wb = _qrot(self.q)                       # body→world
        h    = R_wb.T @ (gate_pos_world - self.p)  # predicted, body frame

        # Jacobian H (3×15):  H = [∂h/∂δp | 0 | ∂h/∂δθ | 0 | 0]
        H = np.zeros((3, self.N))
        H[:, 0:3] = -R_wb.T    # ∂h/∂δp = −R^T
        H[:, 6:9] = _skew(h)   # ∂h/∂δθ = skew(h)

        # Innovation and covariance
        innov = z_body - h
        S     = H @ self.P @ H.T + R_meas_body

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False, innov, float("inf")

        mahal = float(innov @ S_inv @ innov)
        if mahal > self.chi2_thresh:
            return False, innov, mahal

        # Kalman gain and error-state correction
        K  = self.P @ H.T @ S_inv
        dx = K @ innov

        # Inject correction into nominal state
        self.p  += dx[0:3]
        self.v  += dx[3:6]
        self.q   = _qnorm(_qmul(self.q, _rotvec_to_quat(dx[6:9])))
        self.bg += dx[9:12]
        self.ba += dx[12:15]

        # Joseph-form covariance update (numerically stable)
        I_KH    = np.eye(self.N) - K @ H
        self.P  = I_KH @ self.P @ I_KH.T + K @ R_meas_body @ K.T
        self.P  = 0.5 * (self.P + self.P.T)

        return True, innov, mahal
