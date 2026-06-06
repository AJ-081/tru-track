"""
eskf_python.py — Exact Python port of Eskf3D.cpp / EskfConfig.h
Used by tuning_backend.py to replay sessions with custom parameters.

State vector (15 states):
  [0:3]  pE, pN, pU   — position in local ENU (m)
  [3:6]  vE, vN, vU   — velocity in ENU (m/s)
  [6:9]  δφ, δθ, δψ   — attitude error (rad)
  [9:12] bax, bay, baz — accel bias (m/s²)
  [12:15] bgx, bgy, bgz — gyro bias (rad/s)

All matrix ops use numpy. Read-only — no MongoDB writes.
"""

import numpy as np

# ── Earth / physics ───────────────────────────────────────────────────────────
GRAVITY       = 9.80665          # m/s²
EARTH_RADIUS  = 6378137.0        # WGS-84 (m)


# ── Default config matching EskfConfig.h ─────────────────────────────────────
DEFAULT_CFG = dict(
    sigma_acc       = 0.15,      # m/s²/√Hz
    sigma_gyro      = 0.005,     # rad/s/√Hz
    sigma_acc_bias  = 0.0005,    # m/s²/√Hz (random walk)
    sigma_gyro_bias = 0.0002,    # rad/s/√Hz (random walk)
    sigma_gnss_pos  = 3.0,       # m (1-sigma)
    sigma_gnss_vel  = 0.15,      # m/s (1-sigma)
    sigma_gnss_alt  = 2.5,       # m (1-sigma)
    gating_thresh   = 1e9,       # disabled in replay — tune σ params first, then tune gating separately
    use_gnss_vel    = True,
    use_altitude    = True,
    use_nhc         = True,
    use_zupt        = True,
    min_dt          = 0.0005,    # s
    max_dt          = 0.5,       # s — IMU at 5Hz in DB
    zupt_acc_std    = 0.3,       # m/s²
    zupt_gyr_std    = 0.05,      # rad/s
)


# ── Geo utilities ─────────────────────────────────────────────────────────────
class GeoUtils:
    """LLA ↔ ENU conversion (flat-earth around a reference point)."""
    _ref_lat = 0.0
    _ref_lon = 0.0
    _ref_alt = 0.0
    _ref_set = False

    @classmethod
    def set_reference(cls, lat_deg, lon_deg, alt_m):
        cls._ref_lat = np.radians(lat_deg)
        cls._ref_lon = np.radians(lon_deg)
        cls._ref_alt = float(alt_m)
        cls._ref_set = True

    @classmethod
    def lla_to_enu(cls, lat_deg, lon_deg, alt_m):
        lat = np.radians(lat_deg)
        lon = np.radians(lon_deg)
        dlat = lat - cls._ref_lat
        dlon = lon - cls._ref_lon
        E = dlon * EARTH_RADIUS * np.cos(cls._ref_lat)
        N = dlat * EARTH_RADIUS
        U = float(alt_m) - cls._ref_alt
        return float(E), float(N), float(U)

    @classmethod
    def enu_to_lla(cls, E, N, U):
        lat = cls._ref_lat + N / EARTH_RADIUS
        lon = cls._ref_lon + E / (EARTH_RADIUS * np.cos(cls._ref_lat))
        alt = cls._ref_alt + U
        return float(np.degrees(lat)), float(np.degrees(lon)), float(alt)


# ── Quaternion helpers ────────────────────────────────────────────────────────
def quat_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])

def quat_to_Rnb(q):
    qw, qx, qy, qz = q
    return np.array([
        [1-2*(qy*qy+qz*qz),  2*(qx*qy-qz*qw),  2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),    1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),    2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])

def quat_apply_small_angle(dtheta, q):
    """Apply small-angle attitude correction. dtheta = [dtx, dty, dtz]"""
    dq = np.array([1.0, 0.5*dtheta[0], 0.5*dtheta[1], 0.5*dtheta[2]])
    return quat_normalize(quat_mul(dq, q))

def quat_from_yaw(yaw):
    h = 0.5 * yaw
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])

def euler_from_quat(q):
    qw, qx, qy, qz = q
    roll  = np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
    sinp  = 2*(qw*qy-qz*qx)
    sinp  = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    yaw   = np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return float(roll), float(pitch), float(yaw)

def skew3(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

def inv3(M):
    """3×3 matrix inverse. Returns (inv, ok)."""
    det = np.linalg.det(M)
    if abs(det) < 1e-12:
        return None, False
    return np.linalg.inv(M), True


# ── Main ESKF class ───────────────────────────────────────────────────────────
class Eskf3D:
    """
    15-state Error-State Kalman Filter.
    Exact Python port of firmware Eskf3D.cpp.
    """

    def __init__(self, cfg: dict = None):
        self.cfg = {**DEFAULT_CFG, **(cfg or {})}
        self._reset()

    def _reset(self):
        self.pE = self.pN = self.pU = 0.0
        self.vE = self.vN = self.vU = 0.0
        self.q  = np.array([1.0, 0.0, 0.0, 0.0])  # qw, qx, qy, qz
        self.bax = self.bay = self.baz = 0.0
        self.bgx = self.bgy = self.bgz = 0.0
        self.P = np.zeros((15, 15))
        self.last_t_us  = 0
        self.initialized = False
        self.last_pos_innov = np.zeros(3)
        self.last_vel_innov = np.zeros(3)

    def reconfigure(self, cfg: dict):
        """Update parameters and reset. Called when sliders change."""
        self.cfg = {**self.cfg, **cfg}
        self._reset()

    def init_lla(self, lat_deg, lon_deg, alt_m):
        GeoUtils.set_reference(lat_deg, lon_deg, alt_m)
        self.pE = self.pN = self.pU = 0.0
        self.vE = self.vN = self.vU = 0.0
        self.q  = np.array([1.0, 0.0, 0.0, 0.0])
        self.bax = self.bay = self.baz = 0.0
        self.bgx = self.bgy = self.bgz = 0.0
        self.last_t_us   = 0
        self.initialized = True
        self.last_pos_innov = np.zeros(3)
        self.last_vel_innov = np.zeros(3)

        # ── Covariance init (matches firmware) ─────────────────────────────
        P = np.zeros((15, 15))
        P[0,0] = P[1,1] = 25.0          # pos xy  (5m σ)
        P[2,2]           = 64.0          # pos z   (8m σ)
        P[3,3] = P[4,4] = 1.0           # vel xy
        P[5,5]           = 2.25          # vel z
        P[6,6] = P[7,7] = 0.09          # att xy (0.3 rad σ)
        P[8,8]           = 0.25          # att z  (0.5 rad σ)
        P[9,9] = P[10,10] = P[11,11] = 0.25    # accel bias
        P[12,12] = P[13,13] = P[14,14] = 0.0025 # gyro bias
        self.P = P

    def set_initial_yaw(self, yaw_rad):
        if not self.initialized:
            return
        self.q = quat_normalize(quat_from_yaw(yaw_rad))

    # ── Predict ───────────────────────────────────────────────────────────────
    def predict(self, t_us, ax, ay, az, gx, gy, gz):
        if not self.initialized:
            return

        if self.last_t_us == 0:
            self.last_t_us = t_us
            return

        dt = (t_us - self.last_t_us) * 1e-6
        if dt < self.cfg['min_dt'] or dt > self.cfg['max_dt']:
            self.last_t_us = t_us
            return
        self.last_t_us = t_us

        # Bias-compensated IMU
        wx = gx - self.bgx; wy = gy - self.bgy; wz = gz - self.bgz
        fx = ax - self.bax; fy = ay - self.bay; fz = az - self.baz

        # Quaternion integration: q_dot = 0.5 * q ⊗ [0, ω]
        omega_q = np.array([0.0, wx, wy, wz])
        q_dot   = 0.5 * quat_mul(self.q, omega_q)
        self.q  = quat_normalize(self.q + q_dot * dt)

        # Specific force → nav frame
        Rnb = quat_to_Rnb(self.q)
        f_nav = Rnb @ np.array([fx, fy, fz])

        # Velocity integration (ENU: gravity = -g in U)
        self.vE += f_nav[0] * dt
        self.vN += f_nav[1] * dt
        self.vU += (f_nav[2] - GRAVITY) * dt

        # Position integration
        self.pE += self.vE * dt
        self.pN += self.vN * dt
        self.pU += self.vU * dt

        # Covariance predict
        self._predict_P(dt, f_nav, np.array([wx, wy, wz]))

    def _predict_P(self, dt, f_nav, omega):
        """Propagate covariance: P ← F P F^T + Q (matches eskf_predict_P in C++)"""
        cfg = self.cfg
        Rnb = quat_to_Rnb(self.q)
        N   = 15

        # ── F matrix ────────────────────────────────────────────────────────
        F = np.eye(N)

        # dp/dt = v
        F[0:3, 3:6] = np.eye(3) * dt

        # dv/dt contributions
        # from attitude error: -Rnb * [f]× * dt
        F[3:6, 6:9] = -Rnb @ skew3(f_nav) * dt
        # from accel bias: -Rnb * dt
        F[3:6, 9:12] = -Rnb * dt

        # dδθ/dt contributions
        # from attitude error itself: -[ω]× * dt
        F[6:9, 6:9] = np.eye(3) - skew3(omega) * dt
        # from gyro bias: -I * dt
        F[6:9, 12:15] = -np.eye(3) * dt

        # ── Q matrix ────────────────────────────────────────────────────────
        sa2  = cfg['sigma_acc']       ** 2
        sg2  = cfg['sigma_gyro']      ** 2
        sab2 = cfg['sigma_acc_bias']  ** 2
        sgb2 = cfg['sigma_gyro_bias'] ** 2

        Q = np.zeros((N, N))

        # Accel noise drives position and velocity
        qpp = 0.5 * sa2 * dt**3
        qpv = 0.5 * sa2 * dt**2
        qvv =       sa2 * dt

        for i in range(3):
            Q[i,   i]   += qpp
            Q[i,   3+i] += qpv
            Q[3+i, i]   += qpv
            Q[3+i, 3+i] += qvv

        # Gyro noise drives attitude error
        qtt = sg2 * dt
        Q[6,6] += qtt; Q[7,7] += qtt; Q[8,8] += qtt

        # Bias random walks
        qba = sab2 * dt
        qbg = sgb2 * dt
        Q[9,9]   += qba; Q[10,10] += qba; Q[11,11] += qba
        Q[12,12] += qbg; Q[13,13] += qbg; Q[14,14] += qbg

        # P = F P F^T + Q
        FP = F @ self.P
        self.P = FP @ F.T + Q

    # ── GNSS position update ──────────────────────────────────────────────────
    def update_gnss_lla(self, lat_deg, lon_deg, alt_m):
        if not self.initialized:
            return None  # NIS not computed

        E, N, U = GeoUtils.lla_to_enu(lat_deg, lon_deg, alt_m)
        cfg = self.cfg

        r = np.array([E - self.pE, N - self.pN, U - self.pU])
        self.last_pos_innov = r.copy()

        # R matrix
        sp2  = cfg['sigma_gnss_pos'] ** 2
        sa2  = cfg['sigma_gnss_alt'] ** 2 if cfg['use_altitude'] else 1e12

        R = np.diag([sp2, sp2, sa2])

        # H = [I 0 0 0 0]  (selects rows 0:3 of state)
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Gating (Mahalanobis)
        Sinv, ok = inv3(S)
        if not ok:
            return None

        NIS = float(r @ Sinv @ r)
        if NIS > cfg['gating_thresh']:
            return -NIS  # negative = rejected by gating

        # Kalman gain
        K = self.P @ H.T @ Sinv  # (15×3)

        # State update
        dx = K @ r

        self.pE += dx[0]; self.pN += dx[1]; self.pU += dx[2]
        self.vE += dx[3]; self.vN += dx[4]; self.vU += dx[5]
        self.q   = quat_apply_small_angle(dx[6:9], self.q)
        self.bax += dx[9];  self.bay += dx[10]; self.baz += dx[11]
        self.bgx += dx[12]; self.bgy += dx[13]; self.bgz += dx[14]

        # Joseph form covariance update (numerically stable)
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self._symmetrize()

        return NIS

    # ── GNSS velocity update ──────────────────────────────────────────────────
    def update_gnss_vel(self, vE_in, vN_in, vU_in=0.0):
        if not self.cfg['use_gnss_vel'] or not self.initialized:
            return

        r = np.array([vE_in - self.vE, vN_in - self.vN, vU_in - self.vU])
        self.last_vel_innov = r.copy()

        sv2 = self.cfg['sigma_gnss_vel'] ** 2
        R   = np.diag([sv2, sv2, 1e12])  # vertical vel unreliable

        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)

        S    = H @ self.P @ H.T + R
        Sinv, ok = inv3(S)
        if not ok:
            return

        K  = self.P @ H.T @ Sinv
        dx = K @ r

        self.pE += dx[0]; self.pN += dx[1]; self.pU += dx[2]
        self.vE += dx[3]; self.vN += dx[4]; self.vU += dx[5]
        self.q   = quat_apply_small_angle(dx[6:9], self.q)
        self.bax += dx[9];  self.bay += dx[10]; self.baz += dx[11]
        self.bgx += dx[12]; self.bgy += dx[13]; self.bgz += dx[14]

        I_KH  = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self._symmetrize()

    # ── NHC update ────────────────────────────────────────────────────────────
    def update_nhc(self):
        """Non-Holonomic Constraint: lateral + vertical body velocity = 0."""
        if not self.cfg['use_nhc'] or not self.initialized:
            return

        Rnb = quat_to_Rnb(self.q)
        Rbn = Rnb.T
        v_nav = np.array([self.vE, self.vN, self.vU])
        v_body = Rbn @ v_nav

        r = np.array([-v_body[1], -v_body[2]])  # lateral + vertical should be 0
        skew_vb = skew3(v_body)

        H = np.zeros((2, 15))
        # Velocity block: rows 1,2 of Rbn
        H[0, 3:6] = Rbn[1, :]
        H[1, 3:6] = Rbn[2, :]
        # Attitude block: rows 1,2 of [v_body]×
        H[0, 6:9] = skew_vb[1, :]
        H[1, 6:9] = skew_vb[2, :]

        sv = 0.5
        R  = np.diag([sv*sv, sv*sv])

        PHt = self.P @ H.T
        S   = H @ PHt + R
        det = S[0,0]*S[1,1] - S[0,1]*S[1,0]
        if abs(det) < 1e-6:
            return
        Sinv = np.array([[S[1,1], -S[0,1]], [-S[1,0], S[0,0]]]) / det

        K  = PHt @ Sinv
        dx = K @ r

        self.pE += dx[0]; self.pN += dx[1]; self.pU += dx[2]
        self.vE += dx[3]; self.vN += dx[4]; self.vU += dx[5]
        self.q   = quat_apply_small_angle(dx[6:9], self.q)
        self.bax += dx[9];  self.bay += dx[10]; self.baz += dx[11]
        self.bgx += dx[12]; self.bgy += dx[13]; self.bgz += dx[14]

        # Simple update: P = P - KHP  (matches C++ NHC implementation)
        HP  = H @ self.P
        KHP = K @ HP
        self.P -= KHP
        self._symmetrize()

    # ── ZUPT update ───────────────────────────────────────────────────────────
    def update_zupt(self):
        """Zero Velocity Update when device is static."""
        if not self.cfg['use_zupt'] or not self.initialized:
            return

        r  = np.array([-self.vE, -self.vN, -self.vU])
        sz = 0.05
        R  = np.diag([sz*sz, sz*sz, sz*sz])

        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)

        S    = H @ self.P @ H.T + R
        Sinv, ok = inv3(S)
        if not ok:
            return

        K  = self.P @ H.T @ Sinv
        dx = K @ r

        self.vE += dx[3]; self.vN += dx[4]; self.vU += dx[5]

        HP  = H @ self.P
        KHP = K @ HP
        self.P -= KHP
        self._symmetrize()

    def is_static(self, ax, ay, az, gx, gy, gz):
        g_norm = np.sqrt(gx*gx + gy*gy + gz*gz)
        if g_norm > self.cfg['zupt_gyr_std']:
            return False
        a_norm = np.sqrt(ax*ax + ay*ay + az*az)
        if abs(a_norm - GRAVITY) > self.cfg['zupt_acc_std']:
            return False
        return True

    # ── Getters ───────────────────────────────────────────────────────────────
    def get_lla(self):
        return GeoUtils.enu_to_lla(self.pE, self.pN, self.pU)

    def get_velocity(self):
        return self.vE, self.vN, self.vU

    def get_euler(self):
        return euler_from_quat(self.q)

    def get_cov_diag(self):
        return np.diag(self.P).tolist()

    def _symmetrize(self):
        self.P = 0.5 * (self.P + self.P.T)


# ── Replay engine ─────────────────────────────────────────────────────────────
def replay_session(imu_data, gnss_data, cfg, gnss_deny_ranges=None):
    """
    Run ESKF over a stored session with custom parameters.

    imu_data:  list of dicts {t_ms, ax, ay, az, gx, gy, gz}   (100 Hz)
    gnss_data: list of dicts {t_ms, lat, lon, alt, speed, course, fix_valid}  (1 Hz)
    cfg:       dict of ESKF parameters (keys from DEFAULT_CFG)
    gnss_deny_ranges: list of [t_start_ms, t_end_ms] to block GNSS updates

    Returns:
      track:   list of {t_ms, lat, lon, alt, vE, vN, roll, pitch, yaw}
      nis_seq: list of {t_ms, NIS} (one per GNSS update)
      cov_seq: list of {t_ms, P_pos_E, P_pos_N, P_pos_U, P_vel_E, P_vel_N}
      summary: dict with stats
    """
    eskf = Eskf3D(cfg)
    track, nis_seq, cov_seq = [], [], []

    deny_ranges = gnss_deny_ranges or []

    def is_denied(t_ms):
        for a, b in deny_ranges:
            if a <= t_ms <= b:
                return True
        return False

    # Build GNSS lookup by t_ms (nearest)
    gnss_list = sorted(gnss_data, key=lambda x: x['t_ms'])
    gnss_idx  = 0
    gnss_used = set()

    # Find good init fix
    init_gnss = next((g for g in gnss_list if g.get('fix_valid', True)
                      and g.get('lat', 0) != 0), None)
    if not init_gnss:
        return [], [], [], {"error": "No valid GNSS fix found in session"}

    eskf.init_lla(init_gnss['lat'], init_gnss['lon'], init_gnss.get('alt', 0))

    # Velocity init from first two UNIQUE GNSS positions
    unique2 = []
    _pv_lat, _pv_lon = None, None
    for g in gnss_list:
        if g.get('lat') == _pv_lat and g.get('lon') == _pv_lon:
            continue
        _pv_lat, _pv_lon = g.get('lat'), g.get('lon')
        unique2.append(g)
        if len(unique2) >= 5:  # look at first 5 unique to find motion
            break

    # Find first pair with real movement (>1m) for velocity estimate
    eskf.vE = 0.0; eskf.vN = 0.0
    for i in range(len(unique2)-1):
        g0, g1 = unique2[i], unique2[i+1]
        dt_s = (g1['t_ms'] - g0['t_ms']) / 1000.0
        if dt_s < 0.05:
            continue
        # Temporarily set reference to compute ENU
        _saved_lat = GeoUtils._ref_lat
        _saved_lon = GeoUtils._ref_lon
        _saved_alt = GeoUtils._ref_alt
        GeoUtils.set_reference(g0['lat'], g0['lon'], g0.get('alt', 0))
        E1, N1, _ = GeoUtils.lla_to_enu(g1['lat'], g1['lon'], g1.get('alt', 0))
        # Restore reference
        import math as _m
        GeoUtils._ref_lat = _saved_lat
        GeoUtils._ref_lon = _saved_lon
        GeoUtils._ref_alt = _saved_alt
        dist = np.sqrt(E1**2 + N1**2)
        if dist < 0.5:  # < 0.5m movement, skip
            continue
        vE_est = E1 / dt_s
        vN_est = N1 / dt_s
        spd_est = np.sqrt(vE_est**2 + vN_est**2)
        if spd_est < 80:  # sanity < 288 km/h
            eskf.vE = float(vE_est)
            eskf.vN = float(vN_est)
            if spd_est > 0.3:
                eskf.set_initial_yaw(float(np.arctan2(vE_est, vN_est)))
        break

    nis_values = []
    t0 = imu_data[0]['t_ms'] if imu_data else 0

    # Deduplicate GNSS — firmware publishes at 5Hz but GPS updates at 1Hz
    # Same lat/lon appears 5× — keep only when coordinate actually changed
    deduped_gnss = []
    prev_lat, prev_lon = None, None
    for g in gnss_list:
        if not g.get('fix_valid', True) or not g.get('lat'):
            continue
        if g.get('lat') == prev_lat and g.get('lon') == prev_lon:
            continue  # skip duplicate fix
        prev_lat, prev_lon = g.get('lat'), g.get('lon')
        deduped_gnss.append(g)
    gnss_list = deduped_gnss

    for imu in sorted(imu_data, key=lambda x: x['t_ms']):
        t_ms = imu['t_ms']
        t_us = int(t_ms * 1000)

        # IMU predict
        eskf.predict(
            t_us,
            imu.get('ax', 0), imu.get('ay', 0), imu.get('az', 0),
            imu.get('gx', 0), imu.get('gy', 0), imu.get('gz', 0),
        )

        # NHC
        eskf.update_nhc()

        # ZUPT if static
        ax, ay, az = imu.get('ax',0), imu.get('ay',0), imu.get('az',0)
        gx, gy, gz = imu.get('gx',0), imu.get('gy',0), imu.get('gz',0)
        if eskf.is_static(ax, ay, az, gx, gy, gz):
            eskf.update_zupt()

        # GNSS update (check for new GNSS point at this t_ms)
        while gnss_idx < len(gnss_list):
            g = gnss_list[gnss_idx]
            if g['t_ms'] > t_ms:
                break
            if g['t_ms'] not in gnss_used:
                gnss_used.add(g['t_ms'])
                if g.get('fix_valid', True) and g.get('lat', 0) != 0:
                    if not is_denied(g['t_ms']):
                        NIS = eskf.update_gnss_lla(g['lat'], g['lon'], g.get('alt', 0))
                        # Velocity update
                        spd = g.get('speed', 0)
                        crs = g.get('course', 0)
                        if spd and spd > cfg.get('gnss_vel_gate', 0.8):
                            vE_m = spd * np.sin(np.radians(crs))
                            vN_m = spd * np.cos(np.radians(crs))
                            eskf.update_gnss_vel(vE_m, vN_m)
                        if NIS is not None:
                            accepted = NIS >= 0
                            nis_seq.append({'t_ms': t_ms,
                                            'NIS': abs(NIS),
                                            'denied': False,
                                            'gated': not accepted})
                            if accepted:
                                nis_values.append(NIS)
                    else:
                        nis_seq.append({'t_ms': t_ms, 'NIS': None, 'denied': True})
            gnss_idx += 1

        # Record track at every IMU step
        if eskf.initialized:
            lat, lon, alt = eskf.get_lla()
            vE, vN, vU    = eskf.get_velocity()
            roll, pitch, yaw = eskf.get_euler()
            track.append({
                't_ms': t_ms,
                'lat': round(lat, 7), 'lon': round(lon, 7), 'alt': round(alt, 2),
                'vE': round(vE, 3), 'vN': round(vN, 3), 'vU': round(vU, 3),
                'roll': round(roll, 4), 'pitch': round(pitch, 4), 'yaw': round(yaw, 4),
            })

            # Covariance record (every 10 IMU steps to reduce data)
            if len(track) % 10 == 0:
                diag = eskf.get_cov_diag()
                cov_seq.append({
                    't_ms':   t_ms,
                    'P_pos_E': round(diag[0], 4),
                    'P_pos_N': round(diag[1], 4),
                    'P_pos_U': round(diag[2], 4),
                    'P_vel_E': round(diag[3], 4),
                    'P_vel_N': round(diag[4], 4),
                    'P_vel_U': round(diag[5], 4),
                })

    # Summary stats
    mean_NIS = float(np.mean(nis_values)) if nis_values else None
    std_NIS  = float(np.std(nis_values))  if nis_values else None
    summary  = {
        'imu_count':  len(imu_data),
        'gnss_count': len(gnss_list),
        'track_pts':  len(track),
        'nis_count':  len(nis_values),
        'mean_NIS':   round(mean_NIS, 3) if mean_NIS is not None else None,
        'std_NIS':    round(std_NIS,  3) if std_NIS  is not None else None,
        'tuning_verdict': _verdict(mean_NIS),
        'duration_s': (track[-1]['t_ms'] - track[0]['t_ms']) / 1000 if len(track) > 1 else 0,
    }

    return track, nis_seq, cov_seq, summary


def _verdict(mean_NIS):
    if mean_NIS is None:
        return 'no_data'
    if mean_NIS < 1.0:
        return 'overconfident'      # too much process noise
    if mean_NIS < 2.0:
        return 'slightly_loose'
    if mean_NIS < 4.5:
        return 'well_tuned'         # χ²(3) mean ≈ 3
    if mean_NIS < 8.0:
        return 'slightly_tight'
    return 'too_tight'              # too little process noise / too much meas noise


def generate_eskf_config_h(cfg):
    """Generate EskfConfig.h content from tuned parameters."""
    return f"""#ifndef ESKF_CONFIG_H
#define ESKF_CONFIG_H
#define ESKF_PI 3.14159265358979323846f

// ================================================================
// ESKF 3D Configuration — Generated by TRU-TRACK Tuning Dashboard
// ================================================================

// ---------- Compile-time switches ----------
#define ESKF_USE_GNSS_VELOCITY {1 if cfg.get('use_gnss_vel', True) else 0}
#define ESKF_USE_ALTITUDE      {1 if cfg.get('use_altitude', True) else 0}
#define ESKF_USE_BIAS_ESTIMATION 1
#define ESKF_USE_NHC           {1 if cfg.get('use_nhc', True) else 0}
#define ESKF_USE_GATING        1
#define ESKF_GATING_THRESH     {cfg.get('gating_thresh', 10.0):.1f}f

// ---------- Physical constants ----------
#define ESKF_GRAVITY      9.80665f
#define ESKF_EARTH_RADIUS 6378137.0f

// ---------- IMU Noise Parameters ----------
#define ESKF_SIGMA_ACC       {cfg.get('sigma_acc',      0.15):.5f}f
#define ESKF_SIGMA_GYRO      {cfg.get('sigma_gyro',     0.005):.5f}f
#define ESKF_SIGMA_ACC_BIAS  {cfg.get('sigma_acc_bias', 0.0005):.6f}f
#define ESKF_SIGMA_GYRO_BIAS {cfg.get('sigma_gyro_bias',0.0002):.6f}f

// ---------- GNSS Measurement Noise ----------
#define ESKF_SIGMA_GNSS_POS {cfg.get('sigma_gnss_pos', 3.0):.2f}f
#define ESKF_SIGMA_GNSS_VEL {cfg.get('sigma_gnss_vel', 0.15):.3f}f
#define ESKF_SIGMA_GNSS_ALT {cfg.get('sigma_gnss_alt', 2.5):.2f}f

// ---------- Numerical limits ----------
#define ESKF_MAX_DT  {cfg.get('max_dt', 0.2):.3f}f
#define ESKF_MIN_DT  {cfg.get('min_dt', 0.0005):.5f}f

// ---------- ZUPT Thresholds ----------
#define ESKF_ZUPT_ACC_STD {cfg.get('zupt_acc_std', 0.3):.2f}f
#define ESKF_ZUPT_GYR_STD {cfg.get('zupt_gyr_std', 0.05):.3f}f

#endif // ESKF_CONFIG_H
"""
