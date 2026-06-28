#include "Eskf3D.h"
#include "EskfConfig.h"
#include "EskfMath.h"
#include "GeoUtils.h"
#include <math.h>

// ------------------ small helpers (private to this file) ------------------
static inline void quat_normalize(float &qw, float &qx, float &qy, float &qz);
static inline void quat_apply_small_angle(float dtx, float dty, float dtz,
                                          float &qw, float &qx, float &qy,
                                          float &qz);

static void eskf_gnss_vel_update(float *P,
                                 // nominal state refs
                                 float &pE, float &pN, float &pU, float &vE,
                                 float &vN, float &vU, float &qw, float &qx,
                                 float &qy, float &qz, float &bax, float &bay,
                                 float &baz, float &bgx, float &bgy, float &bgz,
                                 // measurement (ENU)
                                 float *innov_vel, float zVE, float zVN,
                                 float zVU) {
  // Measurement: z = [vE vN vU]
  // H = [0 I3 0 0 0] (3x15), selecting states 3..5

  // Residual r = z - h(x)
  float r[3] = {zVE - vE, zVN - vN, zVU - vU};
  if (innov_vel) {
    innov_vel[0] = r[0];
    innov_vel[1] = r[1];
    innov_vel[2] = r[2];
  }

  // R (3x3)
  float sVE2 = ESKF_SIGMA_GNSS_VEL * ESKF_SIGMA_GNSS_VEL;
  float sVN2 = ESKF_SIGMA_GNSS_VEL * ESKF_SIGMA_GNSS_VEL;

  // Vertical velocity is usually worse / often unavailable
  // If you pass zVU=0 because GNSS doesn't provide it, keep this large.
  float sVU2 = 1e12f;
  float Rm[9] = {sVE2, 0.0f, 0.0f, 0.0f, sVN2, 0.0f, 0.0f, 0.0f, sVU2};

  // S = HPH^T + R = P(3:5,3:5) + R
  float S[9] = {
      P[3 * 15 + 3] + Rm[0], P[3 * 15 + 4] + Rm[1], P[3 * 15 + 5] + Rm[2],
      P[4 * 15 + 3] + Rm[3], P[4 * 15 + 4] + Rm[4], P[4 * 15 + 5] + Rm[5],
      P[5 * 15 + 3] + Rm[6], P[5 * 15 + 4] + Rm[7], P[5 * 15 + 5] + Rm[8],
  };

  float Sinv[9];
  if (!EskfMath::inv3(S, Sinv))
    return;

  // PH^T is columns 3..5 of P -> PHt is 15x3
  float PHt[15 * 3];
  for (int i = 0; i < 15; i++) {
    PHt[i * 3 + 0] = P[i * 15 + 3];
    PHt[i * 3 + 1] = P[i * 15 + 4];
    PHt[i * 3 + 2] = P[i * 15 + 5];
  }

  // K = PHt * Sinv  (15x3)
  float K[15 * 3];
  EskfMath::matMul(PHt, 15, 3, Sinv, 3, 3, K);

  // dx = K*r  (15x1)
  float dx[15];
  for (int i = 0; i < 15; i++) {
    dx[i] = K[i * 3 + 0] * r[0] + K[i * 3 + 1] * r[1] + K[i * 3 + 2] * r[2];
  }

  // ---- Inject error into nominal ----
  pE += dx[0];
  pN += dx[1];
  pU += dx[2];
  vE += dx[3];
  vN += dx[4];
  vU += dx[5];

  quat_apply_small_angle(dx[6], dx[7], dx[8], qw, qx, qy, qz);

  bax += dx[9];
  bay += dx[10];
  baz += dx[11];
  bgx += dx[12];
  bgy += dx[13];
  bgz += dx[14];

  // ---- Joseph covariance update ----
  // P = (I-KH) P (I-KH)^T + K R K^T
  // Here KH affects columns 3..5.

  float A[15 * 15];
  EskfMath::matEye(A, 15, 1.0f);
  for (int i = 0; i < 15; i++) {
    A[i * 15 + 3] -= K[i * 3 + 0];
    A[i * 15 + 4] -= K[i * 3 + 1];
    A[i * 15 + 5] -= K[i * 3 + 2];
  }

  float AP[15 * 15];
  float Ptmp[15 * 15];
  EskfMath::matMul(A, 15, 15, P, 15, 15, AP);
  EskfMath::matMulBT(AP, 15, 15, A, 15, 15, Ptmp);

  // Add K R K^T (R diagonal)
  float KR[15 * 3];
  for (int i = 0; i < 15; i++) {
    KR[i * 3 + 0] = K[i * 3 + 0] * sVE2;
    KR[i * 3 + 1] = K[i * 3 + 1] * sVN2;
    KR[i * 3 + 2] = K[i * 3 + 2] * sVU2;
  }
  float KRKt[15 * 15];
  EskfMath::matMulBT(KR, 15, 3, K, 15, 3, KRKt);

  EskfMath::matAddInPlace(Ptmp, KRKt, 15, 15, 1.0f);

  EskfMath::matCopy(Ptmp, P, 15, 15);
  EskfMath::matCopy(Ptmp, P, 15, 15);
}

static inline void quat_apply_small_angle(float dtx, float dty, float dtz,
                                          float &qw, float &qx, float &qy,
                                          float &qz) {
  // Small-angle correction: dq ≈ [1, 0.5*dtheta]
  float dq_w = 1.0f;
  float dq_x = 0.5f * dtx;
  float dq_y = 0.5f * dty;
  float dq_z = 0.5f * dtz;

  float nw, nx, ny, nz;
  // dq ⊗ q
  nw = dq_w * qw - dq_x * qx - dq_y * qy - dq_z * qz;
  nx = dq_w * qx + dq_x * qw + dq_y * qz - dq_z * qy;
  ny = dq_w * qy - dq_x * qz + dq_y * qw + dq_z * qx;
  nz = dq_w * qz + dq_x * qy - dq_y * qx + dq_z * qw;

  qw = nw;
  qx = nx;
  qy = ny;
  qz = nz;
  quat_normalize(qw, qx, qy, qz);
}

static inline void skew3(float x, float y, float z, float S[9]) {
  // S = [  0  -z   y
  //        z   0  -x
  //       -y   x   0 ]
  S[0] = 0.0f;
  S[1] = -z;
  S[2] = y;
  S[3] = z;
  S[4] = 0.0f;
  S[5] = -x;
  S[6] = -y;
  S[7] = x;
  S[8] = 0.0f;
}

static inline void quat_to_Rnb(float qw, float qx, float qy, float qz,
                               float R[9]) {
  // R_nb (body -> nav), row-major 3x3
  float r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
  float r12 = 2.0f * (qx * qy - qz * qw);
  float r13 = 2.0f * (qx * qz + qy * qw);

  float r21 = 2.0f * (qx * qy + qz * qw);
  float r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
  float r23 = 2.0f * (qy * qz - qx * qw);

  float r31 = 2.0f * (qx * qz - qy * qw);
  float r32 = 2.0f * (qy * qz + qx * qw);
  float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

  R[0] = r11;
  R[1] = r12;
  R[2] = r13;
  R[3] = r21;
  R[4] = r22;
  R[5] = r23;
  R[6] = r31;
  R[7] = r32;
  R[8] = r33;
}

static inline void quat_mul(float aw, float ax, float ay, float az, float bw,
                            float bx, float by, float bz, float &ow, float &ox,
                            float &oy, float &oz) {
  // o = a ⊗ b
  ow = aw * bw - ax * bx - ay * by - az * bz;
  ox = aw * bx + ax * bw + ay * bz - az * by;
  oy = aw * by - ax * bz + ay * bw + az * bx;
  oz = aw * bz + ax * by - ay * bx + az * bw;
}

static inline void rotate_body_to_nav(float qw, float qx, float qy, float qz,
                                      float bx, float by, float bz, float &nx,
                                      float &ny, float &nz) {
  float r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
  float r12 = 2.0f * (qx * qy - qz * qw);
  float r13 = 2.0f * (qx * qz + qy * qw);

  float r21 = 2.0f * (qx * qy + qz * qw);
  float r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
  float r23 = 2.0f * (qy * qz - qx * qw);

  float r31 = 2.0f * (qx * qz - qy * qw);
  float r32 = 2.0f * (qy * qz + qx * qw);
  float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

  nx = r11 * bx + r12 * by + r13 * bz;
  ny = r21 * bx + r22 * by + r23 * bz;
  nz = r31 * bx + r32 * by + r33 * bz;
}

static inline float wrap_pi(float a) {
  while (a <= -ESKF_PI)
    a += 2.0f * ESKF_PI;
  while (a > ESKF_PI)
    a -= 2.0f * ESKF_PI;
  return a;
}

static inline void quat_normalize(float &qw, float &qx, float &qy, float &qz) {
  float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n <= 0.0f) {
    qw = 1.0f;
    qx = qy = qz = 0.0f;
    return;
  }
  float inv = 1.0f / n;
  qw *= inv;
  qx *= inv;
  qy *= inv;
  qz *= inv;
}

static inline void quat_from_yaw(float yaw, float &qw, float &qx, float &qy,
                                 float &qz) {
  float h = 0.5f * yaw;
  qw = cosf(h);
  qx = 0.0f;
  qy = 0.0f;
  qz = sinf(h);
}

static inline void euler_from_quat(float qw, float qx, float qy, float qz,
                                   float &roll, float &pitch, float &yaw) {
  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  roll = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (qw * qy - qz * qx);
  if (fabsf(sinp) >= 1.0f)
    pitch = copysignf(0.5f * ESKF_PI, sinp);
  else
    pitch = asinf(sinp);

  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  yaw = atan2f(siny_cosp, cosy_cosp);
  yaw = wrap_pi(yaw);
}

// ----------------------------------------------------------------------------
// Non-Holonomic Constraint (NHC) Update
// ----------------------------------------------------------------------------
// Pseudo-measurement: Body-frame velocity v_y = 0, v_z = 0
// H (2x15) = [ 0 | R_nb^T | 0 | 0 | 0 ]
static void eskf_nhc_update(float *P, float &pE, float &pN, float &pU,
                            float &vE, float &vN, float &vU, float &qw,
                            float &qx, float &qy, float &qz, float &bax,
                            float &bay, float &baz, float &bgx, float &bgy,
                            float &bgz) {
  // 1. Calculate nominal body velocity
  float Rnb[9];
  quat_to_Rnb(qw, qx, qy, qz, Rnb);

  // R_bn = R_nb^T
  float Rbn[9];
  EskfMath::matTranspose(Rnb, 3, 3, Rbn);

  // v_body = Rbn * v_nav
  float v_nav[3] = {vE, vN, vU};
  float v_body[3];
  EskfMath::matMul(Rbn, 3, 3, v_nav, 3, 1, v_body);

  // Residual: z_meas - h(x). z_meas = [0, 0] for Y and Z.
  // h(x) is v_body.y and v_body.z
  // r = [ 0 - v_body.y ]
  //     [ 0 - v_body.z ]
  float r[2] = {-v_body[1], -v_body[2]};

  // H matrix (2x15). We only fill non-zero blocks.
  // Block w.r.t Velocity (indices 3,4,5): Rbn (rows 1 and 2)
  // Block w.r.t Theta    (indices 6,7,8): [v_body]x (rows 1 and 2)

  // Jacobian w.r.t error state dtheta is [v_body]x.
  float S_vb[9];
  skew3(v_body[0], v_body[1], v_body[2], S_vb);

  float H[2 * 15];
  EskfMath::matZero(H, 2, 15);

  // Fill H_v part (cols 3,4,5) -> Rbn rows 1,2
  H[0 * 15 + 3] = Rbn[1 * 3 + 0];
  H[0 * 15 + 4] = Rbn[1 * 3 + 1];
  H[0 * 15 + 5] = Rbn[1 * 3 + 2];
  H[1 * 15 + 3] = Rbn[2 * 3 + 0];
  H[1 * 15 + 4] = Rbn[2 * 3 + 1];
  H[1 * 15 + 5] = Rbn[2 * 3 + 2];

  // Fill H_theta part (cols 6,7,8) -> S_vb rows 1,2
  H[0 * 15 + 6] = S_vb[1 * 3 + 0];
  H[0 * 15 + 7] = S_vb[1 * 3 + 1];
  H[0 * 15 + 8] = S_vb[1 * 3 + 2];
  H[1 * 15 + 6] = S_vb[2 * 3 + 0];
  H[1 * 15 + 7] = S_vb[2 * 3 + 1];
  H[1 * 15 + 8] = S_vb[2 * 3 + 2];

  // Measurement noise R (2x2)
  // Tuning: How strict is "no slip"?
  float sigma_vel_y = 0.5f; // m/s allow some slip
  float sigma_vel_z = 0.5f; // m/s allow some bounce
  float R_nhc[4] = {sigma_vel_y * sigma_vel_y, 0, 0, sigma_vel_z * sigma_vel_z};

  // Innovation covariance S = H P H^T + R
  // PH^T (15x2)
  float PHt[15 * 2];
  float Ht[15 * 2];
  EskfMath::matTranspose(H, 2, 15, Ht);
  EskfMath::matMul(P, 15, 15, Ht, 15, 2, PHt);

  float HPHt[2 * 2];
  EskfMath::matMul(H, 2, 15, PHt, 15, 2, HPHt);

  float S[4] = {HPHt[0] + R_nhc[0], HPHt[1] + R_nhc[1], HPHt[2] + R_nhc[2],
                HPHt[3] + R_nhc[3]};

  // Invert S (2x2)
  float det = S[0] * S[3] - S[1] * S[2];
  if (fabsf(det) < 1e-6f)
    return;
  float invDet = 1.0f / det;
  float Sinv[4] = {S[3] * invDet, -S[1] * invDet, -S[2] * invDet,
                   S[0] * invDet};

  // Kalman Gain K = PHt * Sinv (15x2)
  float K[15 * 2];
  EskfMath::matMul(PHt, 15, 2, Sinv, 2, 2, K);

  // Update State dx = K * r
  float dx[15];
  for (int i = 0; i < 15; i++) {
    dx[i] = K[i * 2 + 0] * r[0] + K[i * 2 + 1] * r[1];
  }

  // Apply injection
  pE += dx[0];
  pN += dx[1];
  pU += dx[2];
  vE += dx[3];
  vN += dx[4];
  vU += dx[5];
  quat_apply_small_angle(dx[6], dx[7], dx[8], qw, qx, qy, qz);
  bax += dx[9];
  bay += dx[10];
  baz += dx[11];
  bgx += dx[12];
  bgy += dx[13];
  bgz += dx[14];

  // Update Covariance P = (I - KH)P
  // Using P = P - K*H*P
  float HP[2 * 15];
  EskfMath::matMul(H, 2, 15, P, 15, 15, HP);
  float KHP[15 * 15];
  EskfMath::matMul(K, 15, 2, HP, 2, 15, KHP);

  for (int i = 0; i < 15 * 15; i++)
    P[i] -= KHP[i];

  // Force symmetry
  for (int i = 0; i < 15; i++) {
    for (int j = i + 1; j < 15; j++) {
      float val = 0.5f * (P[i * 15 + j] + P[j * 15 + i]);
      P[i * 15 + j] = val;
      P[j * 15 + i] = val;
    }
  }
}

// ----------------------------------------------------------------------------
// Zero Velocity Update (ZUPT)
// ----------------------------------------------------------------------------
// Measurement: v = [0, 0, 0]
// H = [ 0 | I3 | 0 | 0 | 0 ]
static void eskf_zupt_update(float *P, float &vE, float &vN, float &vU) {
  // Residual r = 0 - v
  float r[3] = {-vE, -vN, -vU};

  // H selects indices 3,4,5.
  // R (measurement noise) - very small, we depend on static condition
  float sZupt = 0.05f; // 0.05 m/s uncertainty
  float R[3] = {sZupt * sZupt, sZupt * sZupt, sZupt * sZupt};

  // S = HPH^T + R = P(3:5, 3:5) + R
  float S[9] = {
      P[3 * 15 + 3] + R[0], P[3 * 15 + 4],        P[3 * 15 + 5],
      P[4 * 15 + 3],        P[4 * 15 + 4] + R[1], P[4 * 15 + 5],
      P[5 * 15 + 3],        P[5 * 15 + 4],        P[5 * 15 + 5] + R[2]};

  float Sinv[9];
  if (!EskfMath::inv3(S, Sinv))
    return;

  // PH^T is columns 3..5 of P
  float PHt[15 * 3];
  for (int i = 0; i < 15; i++) {
    PHt[i * 3 + 0] = P[i * 15 + 3];
    PHt[i * 3 + 1] = P[i * 15 + 4];
    PHt[i * 3 + 2] = P[i * 15 + 5];
  }

  // K = PHt * Sinv
  float K[15 * 3];
  EskfMath::matMul(PHt, 15, 3, Sinv, 3, 3, K);

  // dx = K * r
  float dx[15];
  for (int i = 0; i < 15; i++) {
    dx[i] = K[i * 3 + 0] * r[0] + K[i * 3 + 1] * r[1] + K[i * 3 + 2] * r[2];
  }

  // State Update
  vE += dx[3];
  vN += dx[4];
  vU += dx[5];

  // Covariance Update
  // P = P - K*H*P
  float HP[3 * 15];
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 15; c++) {
      HP[r * 15 + c] = P[(3 + r) * 15 + c];
    }
  }
  float KHP[15 * 15];
  EskfMath::matMul(K, 15, 3, HP, 3, 15, KHP);

  for (int i = 0; i < 15 * 15; i++)
    P[i] -= KHP[i];

  // Force Symmetry
  for (int i = 0; i < 15; i++) {
    for (int j = i + 1; j < 15; j++) {
      float val = 0.5f * (P[i * 15 + j] + P[j * 15 + i]);
      P[i * 15 + j] = val;
      P[j * 15 + i] = val;
    }
  }
}

// Returns true if the update was applied, false if rejected by gating
// (Mahalanobis outlier check). This lets the caller report exactly what
// happened this cycle instead of it being an invisible internal decision.
static bool eskf_gnss_pos_update(float *P,
                                 // nominal state refs
                                 float &pE, float &pN, float &pU, float &vE,
                                 float &vN, float &vU, float &qw, float &qx,
                                 float &qy, float &qz, float &bax, float &bay,
                                 float &baz, float &bgx, float &bgy, float &bgz,
                                 // measurement (ENU)
                                 float *innov_pos, float zE, float zN,
                                 float zU) {
  // Measurement: z = [pE pN pU]
  // H = [I3 0 0 0 0] (3x15)

  // Residual r = z - h(x)
  float r[3] = {zE - pE, zN - pN, zU - pU};
  if (innov_pos) {
    innov_pos[0] = r[0];
    innov_pos[1] = r[1];
    innov_pos[2] = r[2];
  }

  // R (3x3)
  float sE2 = ESKF_SIGMA_GNSS_POS * ESKF_SIGMA_GNSS_POS;
  float sN2 = ESKF_SIGMA_GNSS_POS * ESKF_SIGMA_GNSS_POS;
#if ESKF_USE_ALTITUDE
  float sU2 = ESKF_SIGMA_GNSS_ALT * ESKF_SIGMA_GNSS_ALT;
#else
  float sU2 = 1e12f;
#endif

  float Rm[9] = {sE2, 0.0f, 0.0f, 0.0f, sN2, 0.0f, 0.0f, 0.0f, sU2};

  // S = HPH^T + R = P(0:2,0:2) + R
  float S[9] = {
      P[0 * 15 + 0] + Rm[0], P[0 * 15 + 1] + Rm[1], P[0 * 15 + 2] + Rm[2],
      P[1 * 15 + 0] + Rm[3], P[1 * 15 + 1] + Rm[4], P[1 * 15 + 2] + Rm[5],
      P[2 * 15 + 0] + Rm[6], P[2 * 15 + 1] + Rm[7], P[2 * 15 + 2] + Rm[8],
  };

  float Sinv[9];
  if (!EskfMath::inv3(S, Sinv))
    return false;   // singular S — cannot update; treat as not-applied

#if ESKF_USE_GATING
  // Mahalanobis distance check: d = r^T * S^-1 * r
  float d2 = 0.0f;
  // 1x3 * 3x3 * 3x1
  // temp = r^T * Sinv (1x3)
  float rtSinv[3];
  rtSinv[0] = r[0] * Sinv[0] + r[1] * Sinv[3] + r[2] * Sinv[6];
  rtSinv[1] = r[0] * Sinv[1] + r[1] * Sinv[4] + r[2] * Sinv[7];
  rtSinv[2] = r[0] * Sinv[2] + r[1] * Sinv[5] + r[2] * Sinv[8];

  d2 = rtSinv[0] * r[0] + rtSinv[1] * r[1] + rtSinv[2] * r[2];

  if (d2 > (ESKF_GATING_THRESH * ESKF_GATING_THRESH)) {
    // Outlier rejected — caller (firmware telemetry) needs to know this
    // happened, so it's a return value now instead of a silent skip.
    return false;
  }
#endif

  // PH^T is first 3 columns of P -> PHt is 15x3
  float PHt[15 * 3];
  for (int i = 0; i < 15; i++) {
    PHt[i * 3 + 0] = P[i * 15 + 0];
    PHt[i * 3 + 1] = P[i * 15 + 1];
    PHt[i * 3 + 2] = P[i * 15 + 2];
  }

  // K = PHt * Sinv  (15x3)
  float K[15 * 3];
  EskfMath::matMul(PHt, 15, 3, Sinv, 3, 3, K);

  // dx = K*r  (15x1)
  float dx[15];
  for (int i = 0; i < 15; i++) {
    dx[i] = K[i * 3 + 0] * r[0] + K[i * 3 + 1] * r[1] + K[i * 3 + 2] * r[2];
  }

  // ---- Inject error into nominal ----
  // dp
  pE += dx[0];
  pN += dx[1];
  pU += dx[2];
  // dv
  vE += dx[3];
  vN += dx[4];
  vU += dx[5];
  // dtheta
  quat_apply_small_angle(dx[6], dx[7], dx[8], qw, qx, qy, qz);
  // dba
  bax += dx[9];
  bay += dx[10];
  baz += dx[11];
  // dbg
  bgx += dx[12];
  bgy += dx[13];
  bgz += dx[14];

  // ---- Joseph covariance update ---- assurance
  // P = (I-KH) P (I-KH)^T + K R K^T
  // Since H selects first 3 states, KH has only first 3 columns equal to K.

  float A[15 * 15];
  EskfMath::matEye(A, 15, 1.0f);
  for (int i = 0; i < 15; i++) {
    // subtract K into first 3 columns
    A[i * 15 + 0] -= K[i * 3 + 0];
    A[i * 15 + 1] -= K[i * 3 + 1];
    A[i * 15 + 2] -= K[i * 3 + 2];
  }

  float AP[15 * 15];
  float Ptmp[15 * 15];
  EskfMath::matMul(A, 15, 15, P, 15, 15, AP);
  EskfMath::matMulBT(AP, 15, 15, A, 15, 15, Ptmp); // AP * A^T

  // Add K R K^T (R is diagonal here)
  float KR[15 * 3];
  for (int i = 0; i < 15; i++) {
    KR[i * 3 + 0] = K[i * 3 + 0] * sE2;
    KR[i * 3 + 1] = K[i * 3 + 1] * sN2;
    KR[i * 3 + 2] = K[i * 3 + 2] * sU2;
  }
  float KRKt[15 * 15];
  EskfMath::matMulBT(KR, 15, 3, K, 15, 3, KRKt);

  EskfMath::matAddInPlace(Ptmp, KRKt, 15, 15, 1.0f);

  // Copy back
  EskfMath::matCopy(Ptmp, P, 15, 15);

  return true;   // update applied successfully
}

static void eskf_predict_P(float *P, float dt, float qw, float qx, float qy,
                           float qz, float fx, float fy, float fz, float wx,
                           float wy, float wz) {
  // Error-state order:
  // 0-2: dp, 3-5: dv, 6-8: dtheta, 9-11: dba, 12-14: dbg

  const int N = 15;

  // ---- Build F = I + Fc*dt (discrete approx) ----
  float F[N * N];
  EskfMath::matEye(F, N, 1.0f);

  // dp = dp + dv*dt
  F[0 * N + 3] = dt;
  F[1 * N + 4] = dt;
  F[2 * N + 5] = dt;

  // R_nb from quaternion
  float Rnb[9];
  quat_to_Rnb(qw, qx, qy, qz, Rnb);

  // Specific force in nav frame (used in dv/dtheta coupling)
  float fE, fN, fU;
  // reuse the same rotation math as rotate_body_to_nav but local:
  {
    float r11 = Rnb[0], r12 = Rnb[1], r13 = Rnb[2];
    float r21 = Rnb[3], r22 = Rnb[4], r23 = Rnb[5];
    float r31 = Rnb[6], r32 = Rnb[7], r33 = Rnb[8];
    fE = r11 * fx + r12 * fy + r13 * fz;
    fN = r21 * fx + r22 * fy + r23 * fz;
    fU = r31 * fx + r32 * fy + r33 * fz;
  }

  // dv = dv - skew(f_nav)*dtheta*dt  - Rnb*dba*dt
  float Sf[9];
  skew3(fE, fN, fU, Sf);

  // -Sf*dt into F[v,theta]
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      F[(3 + i) * N + (6 + j)] += (-Sf[i * 3 + j]) * dt;
    }
  }

  // -Rnb*dt into F[v,ba]
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      F[(3 + i) * N + (9 + j)] += (-Rnb[i * 3 + j]) * dt;
    }
  }

  // dtheta = dtheta - skew(w)*dtheta*dt - dbg*dt
  float Sw[9];
  skew3(wx, wy, wz, Sw);

  // (I - Sw*dt) into F[theta,theta]
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      F[(6 + i) * N + (6 + j)] += (-Sw[i * 3 + j]) * dt;
    }
  }

  // -I*dt into F[theta,bg]
  F[6 * N + 12] = -dt;
  F[7 * N + 13] = -dt;
  F[8 * N + 14] = -dt;

  // Bias random walk: ba_k+1 = ba_k, bg_k+1 = bg_k (already identity in F)

  // ---- Build Q (discrete) ----
  float Q[N * N];
  EskfMath::matZero(Q, N, N);

  const float sa2 = ESKF_SIGMA_ACC * ESKF_SIGMA_ACC;   // (m/s^2)^2 / Hz
  const float sg2 = ESKF_SIGMA_GYRO * ESKF_SIGMA_GYRO; // (rad/s)^2 / Hz
  const float sab2 =
      ESKF_SIGMA_ACC_BIAS * ESKF_SIGMA_ACC_BIAS; // (m/s^2)^2 / Hz
  const float sgb2 =
      ESKF_SIGMA_GYRO_BIAS * ESKF_SIGMA_GYRO_BIAS; // (rad/s)^2 / Hz

  // Discrete noise for integrated accel:
  // Qpp = (dt^3/3)*sa2, Qpv = (dt^2/2)*sa2, Qvv = dt*sa2
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;

  float qpp = (dt3 / 3.0f) * sa2;
  float qpv = (dt2 / 2.0f) * sa2;
  float qvv = (dt)*sa2;

  for (int i = 0; i < 3; i++) {
    Q[(0 + i) * N + (0 + i)] += qpp;
    Q[(0 + i) * N + (3 + i)] += qpv;
    Q[(3 + i) * N + (0 + i)] += qpv;
    Q[(3 + i) * N + (3 + i)] += qvv;
  }

  // Gyro noise drives attitude error: Q_theta = dt*sg2
  float qtt = dt * sg2;
  Q[6 * N + 6] += qtt;
  Q[7 * N + 7] += qtt;
  Q[8 * N + 8] += qtt;

  // Bias random walks: Q_ba = dt*sab2, Q_bg = dt*sgb2
  float qba = dt * sab2;
  float qbg = dt * sgb2;
  Q[9 * N + 9] += qba;
  Q[10 * N + 10] += qba;
  Q[11 * N + 11] += qba;

  Q[12 * N + 12] += qbg;
  Q[13 * N + 13] += qbg;
  Q[14 * N + 14] += qbg;

  // ---- P = F P F^T + Q ----
  float FP[N * N];
  float Pnew[N * N];

  EskfMath::matMul(F, N, N, P, N, N, FP);
  EskfMath::matMulBT(FP, N, N, F, N, N, Pnew); // FP * F^T

  // add Q
  EskfMath::matAddInPlace(Pnew, Q, N, N, 1.0f);

  // copy back
  EskfMath::matCopy(Pnew, P, N, N);
}

// ------------------------------ Eskf3D -----------------------------------

Eskf3D::Eskf3D()
    : pE(0.0f), pN(0.0f), pU(0.0f), vE(0.0f), vN(0.0f), vU(0.0f), qw(1.0f),
      qx(0.0f), qy(0.0f), qz(0.0f), bax(0.0f), bay(0.0f), baz(0.0f), bgx(0.0f),
      bgy(0.0f), bgz(0.0f), last_t_us(0), initialized(false), lat0_deg(0.0f),
      lon0_deg(0.0f), alt0_m(0.0f) {
  // OK to call functions in the constructor body:
  EskfMath::matZero(P, ESKF_NX, ESKF_NX);
}

void Eskf3D::initLLA(float lat_deg, float lon_deg, float alt_m) {
  for (int i = 0; i < 3; i++) {
    last_pos_innov[i] = 0.0f;
    last_vel_innov[i] = 0.0f;
  }
  lat0_deg = lat_deg;
  lon0_deg = lon_deg;
  alt0_m = alt_m;

  GeoUtils::setReferenceLLA(lat0_deg, lon0_deg, alt0_m);

  // Reset nominal state
  pE = pN = pU = 0.0f;
  vE = vN = vU = 0.0f;
  qw = 1.0f;
  qx = 0.0f;
  qy = 0.0f;
  qz = 0.0f;

#if ESKF_USE_BIAS_ESTIMATION
  bax = bay = baz = 0.0f;
  bgx = bgy = bgz = 0.0f;
#else
  bax = bay = baz = 0.0f;
  bgx = bgy = bgz = 0.0f;
#endif

  last_t_us = 0;
  initialized = true;

  // ---------- Covariance initialization ----------
  // Order: [ p(3), v(3), theta(3), ba(3), bg(3) ]
  EskfMath::matZero(P, ESKF_NX, ESKF_NX);

  // Position (m^2)
  P[0 * 15 + 0] = 25.0f; // 5m^2
  P[1 * 15 + 1] = 25.0f;
  P[2 * 15 + 2] = 64.0f; // 8m^2 (vertical worse)

  // Velocity ((m/s)^2)
  P[3 * 15 + 3] = 1.0f;
  P[4 * 15 + 4] = 1.0f;
  P[5 * 15 + 5] = 2.25f; // 1.5^2

  // Attitude (rad^2)
  P[6 * 15 + 6] = 0.09f; // 0.3^2
  P[7 * 15 + 7] = 0.09f;
  P[8 * 15 + 8] = 0.25f; // 0.5^2

  // Accel bias ((m/s^2)^2)
  P[9 * 15 + 9] = 0.25f; // 0.5^2
  P[10 * 15 + 10] = 0.25f;
  P[11 * 15 + 11] = 0.25f;

  // Gyro bias ((rad/s)^2)
  P[12 * 15 + 12] = 0.0025f; // 0.05^2
  P[13 * 15 + 13] = 0.0025f;
  P[14 * 15 + 14] = 0.0025f;
}

void Eskf3D::setInitialYaw(float yaw_rad) {
  if (!initialized)
    return;
  quat_from_yaw(yaw_rad, qw, qx, qy, qz);
  quat_normalize(qw, qx, qy, qz);
}

void Eskf3D::setInitialAttitude(float roll_rad, float pitch_rad, float yaw_rad) {
  if (!initialized)
    return;
  // ZYX Euler → quaternion (yaw first, then pitch, then roll)
  // q = q_yaw ⊗ q_pitch ⊗ q_roll
  float cr=cosf(roll_rad*0.5f),  sr=sinf(roll_rad*0.5f);
  float cp=cosf(pitch_rad*0.5f), sp=sinf(pitch_rad*0.5f);
  float cy=cosf(yaw_rad*0.5f),   sy=sinf(yaw_rad*0.5f);
  qw = cr*cp*cy + sr*sp*sy;
  qx = sr*cp*cy - cr*sp*sy;
  qy = cr*sp*cy + sr*cp*sy;
  qz = cr*cp*sy - sr*sp*cy;
  quat_normalize(qw, qx, qy, qz);
}

void Eskf3D::predict(uint64_t t_us, float ax, float ay, float az, float gx,
                     float gy, float gz) {
  if (!initialized)
    return;

  if (last_t_us == 0) {
    last_t_us = t_us;
    return;
  }

  float dt = (float)((int64_t)t_us - (int64_t)last_t_us) * 1e-6f;
  if (dt < ESKF_MIN_DT)
    return;

  if (dt > ESKF_MAX_DT) {
    last_t_us = t_us;
    return;
  }
  last_t_us = t_us;

#if ESKF_USE_BIAS_ESTIMATION
  float wx = gx - bgx, wy = gy - bgy, wz = gz - bgz;
  float fx = ax - bax, fy = ay - bay, fz = az - baz;
#else
  float wx = gx, wy = gy, wz = gz;
  float fx = ax, fy = ay, fz = az;
#endif

  // q_dot = 0.5 * q ⊗ [0, w]
  float dq_w, dq_x, dq_y, dq_z;
  quat_mul(qw, qx, qy, qz, 0.0f, wx, wy, wz, dq_w, dq_x, dq_y, dq_z);

  qw += 0.5f * dq_w * dt;
  qx += 0.5f * dq_x * dt;
  qy += 0.5f * dq_y * dt;
  qz += 0.5f * dq_z * dt;
  quat_normalize(qw, qx, qy, qz);

  float fE, fN, fU;
  rotate_body_to_nav(qw, qx, qy, qz, fx, fy, fz, fE, fN, fU);

  // ENU: gravity is -g in U
  float aE = fE;
  float aN = fN;
  float aU = fU - ESKF_GRAVITY;

  vE += aE * dt;
  vN += aN * dt;
  vU += aU * dt;

  pE += vE * dt;
  pN += vN * dt;
  pU += vU * dt;

  // ---- Covariance predict (real ESKF starts here) ----
  eskf_predict_P(P, dt, qw, qx, qy, qz, fx, fy, fz, wx, wy, wz);
}

EskfGnssStatus Eskf3D::updateGnssLLA(uint64_t t_us, float lat_deg, float lon_deg,
                           float alt_m) {
  (void)t_us;
  if (!initialized)
    return EskfGnssStatus::SKIPPED;

  float E, N, U;
  GeoUtils::llaToEnu(lat_deg, lon_deg, alt_m, E, N, U);

  bool applied = eskf_gnss_pos_update(P, pE, pN, pU, vE, vN, vU, qw, qx, qy, qz,
                       bax, bay, baz, bgx, bgy, bgz, last_pos_innov, E, N, U);
  return applied ? EskfGnssStatus::APPLIED : EskfGnssStatus::GATED;
}

void Eskf3D::updateGnssVel(float vE_in, float vN_in, float vU_in) {
#if ESKF_USE_GNSS_VELOCITY
  if (!initialized)
    return;

  eskf_gnss_vel_update(P, pE, pN, pU, vE, vN, vU, qw, qx, qy, qz, bax, bay, baz,
                       bgx, bgy, bgz, last_vel_innov, vE_in, vN_in, vU_in);
#else
  (void)vE_in;
  (void)vN_in;
  (void)vU_in;
#endif
}

bool Eskf3D::updateNonHolonomic() {
#if ESKF_USE_NHC
  if (!initialized)
    return false;
  eskf_nhc_update(P, pE, pN, pU, vE, vN, vU, qw, qx, qy, qz, bax, bay, baz, bgx,
                  bgy, bgz);
  return true;
#else
  return false;
#endif
}

bool Eskf3D::updateZeroVelocity() {
  if (!initialized)
    return false;
  eskf_zupt_update(P, vE, vN, vU);
  return true;
}

void Eskf3D::getGyroBias(float &bgx_out, float &bgy_out, float &bgz_out) const {
  bgx_out = bgx;
  bgy_out = bgy;
  bgz_out = bgz;
}

void Eskf3D::getAccelBias(float &bax_out, float &bay_out, float &baz_out) const {
  bax_out = bax;
  bay_out = bay;
  baz_out = baz;
}

bool Eskf3D::isStatic(float ax, float ay, float az, float gx, float gy,
                      float gz) {
  // Simple threshold check on raw values (or calibrated if passed in?)
  // Arguments are typically calibrated (m/s^2, rad/s)

  // Check Gyro (rotation is best indicator)
  float g_norm = sqrtf(gx * gx + gy * gy + gz * gz);
  if (g_norm > ESKF_ZUPT_GYR_STD)
    return false;

  // Check Accel (variance from 1G, or just variance?)
  // If we are moving, we might see 1G + accel.
  // Standard deviation check requires buffer.
  // Instantaneous check: |norm(a) - G| < thresh?
  // No, linear accel can be small.
  // Best single-sample check is just Gyro + low accel dynamic.
  // We can't do std-dev without a buffer.

  // Let's rely on Gyro primarily.
  // And maybe check if Accel norm is close to Gravity (within margin).
  float a_norm = sqrtf(ax * ax + ay * ay + az * az);
  if (fabsf(a_norm - ESKF_GRAVITY) > ESKF_ZUPT_ACC_STD)
    return false;

  return true;
}

void Eskf3D::getLLA(float &lat_deg, float &lon_deg, float &alt_m) const {
  GeoUtils::enuToLla(pE, pN, pU, lat_deg, lon_deg, alt_m);
}

void Eskf3D::getVelocity(float &vE_out, float &vN_out, float &vU_out) const {
  vE_out = vE;
  vN_out = vN;
  vU_out = vU;
}

void Eskf3D::getEuler(float &roll, float &pitch, float &yaw) const {
  euler_from_quat(qw, qx, qy, qz, roll, pitch, yaw);
}

void Eskf3D::getLastPosInnovation(float &dE, float &dN, float &dU) const {
  dE = last_pos_innov[0];
  dN = last_pos_innov[1];
  dU = last_pos_innov[2];
}

void Eskf3D::getLastVelInnovation(float &dVE, float &dVN, float &dVU) const {
  dVE = last_vel_innov[0];
  dVN = last_vel_innov[1];
  dVU = last_vel_innov[2];
}

void Eskf3D::getCovDiag(float diag[15]) const {
  for (int i = 0; i < 15; i++) {
    diag[i] = P[i * 15 + i];
  }
}
