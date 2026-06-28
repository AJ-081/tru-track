#ifndef ESKF3D_H
#define ESKF3D_H

#include <stdint.h>

static constexpr int ESKF_NX = 15;

// Status returned by updateGnssLLA() — lets the caller (firmware
// telemetry) know exactly what happened this cycle, instead of the
// update being a black box. Needed to populate gnss_pos_applied /
// gnss_pos_gated in telemetry and diagnose gating-rejection cascades.
enum class EskfGnssStatus : uint8_t {
  SKIPPED = 0,   // not initialized yet — update did nothing
  APPLIED = 1,   // innovation passed gating, state updated
  GATED   = 2,   // innovation REJECTED by Mahalanobis gate (outlier)
};

// ======================================================
// 3D Error-State Kalman Filter (Vehicle Navigation)
// Internal frame: ENU
// External interface: LLA in / LLA out
// ======================================================

class Eskf3D {
public:
  // ---------- Constructor ----------
  Eskf3D();

  // ---------- Initialization ----------
  // Set ENU origin using GNSS reference
  void initLLA(float lat_deg, float lon_deg, float alt_m);

  // Optional initial yaw (rad, ENU frame)
  void setInitialYaw(float yaw_rad);
  // Initialize full attitude from roll/pitch (gravity-derived) + yaw (GNSS course).
  // Use this instead of setInitialYaw when the device may be tilted.
  // roll_rad: rotation around forward axis, pitch_rad: rotation around lateral axis,
  // yaw_rad: heading (pi/2 - course_deg * pi/180 in ENU frame).
  void setInitialAttitude(float roll_rad, float pitch_rad, float yaw_rad);

  // ---------- Prediction (IMU) ----------
  // ax, ay, az : m/s^2 (specific force, body frame)
  // gx, gy, gz : rad/s   (body frame)
  // t_us       : timestamp in microseconds
  void predict(uint64_t t_us, float ax, float ay, float az, float gx, float gy,
               float gz);

  // ---------- GNSS Update ----------
  // Position update using LLA. Returns whether the update was applied,
  // gated (rejected as an outlier), or skipped (filter not initialized).
  EskfGnssStatus updateGnssLLA(uint64_t t_us, float lat_deg, float lon_deg, float alt_m);

  // Optional GNSS velocity update (ENU)
  void updateGnssVel(float vE, float vN, float vU);

  // ---------- Non-Holonomic Update ----------
  // Force lateral/vertical body velocity to zero. Call after predict().
  // Returns true if the update actually ran (filter was initialized),
  // false if skipped.
  bool updateNonHolonomic();

  // ---------- Zero Velocity Update (ZUPT) ----------
  // Force ALL velocity to zero (vehicle stopped). Returns true if the
  // update actually ran, false if skipped (filter not initialized).
  bool updateZeroVelocity();

  // Helper to detect static condition
  // Returns true if accel/gyro variance is low enough
  bool isStatic(float ax, float ay, float az, float gx, float gy, float gz);

  // ---------- Outputs ----------
  // Position output (LLA)
  void getLLA(float &lat_deg, float &lon_deg, float &alt_m) const;

  // Velocity output (ENU, m/s)
  void getVelocity(float &vE, float &vN, float &vU) const;

  // Attitude output (rad)
  void getEuler(float &roll, float &pitch, float &yaw) const;
  void getLastPosInnovation(float &dE, float &dN, float &dU) const;
  void getLastVelInnovation(float &dVE, float &dVN, float &dVU) const;
  void getCovDiag(float diag[15]) const;

  // Bias outputs — gyro bias already existed via internal state; this
  // adds the matching accel-bias accessor so a runaway accel bias
  // (a real IMU-mistuning failure mode) is visible in telemetry.
  void getGyroBias(float &bgx_out, float &bgy_out, float &bgz_out) const;
  void getAccelBias(float &bax_out, float &bay_out, float &baz_out) const;

private:
  // ---------- Covariance ----------
  // 15x15 covariance, row-major
  float P[ESKF_NX * ESKF_NX];

  // ---------- Nominal State ----------
  // Position (ENU, meters)
  float pE, pN, pU;

  // Velocity (ENU, m/s)
  float vE, vN, vU;

  // Attitude quaternion (body -> nav)
  float qw, qx, qy, qz;

  // IMU biases
  float bax, bay, baz;
  float bgx, bgy, bgz;

  // ---------- Time ----------
  uint64_t last_t_us;
  bool initialized;

  // ---------- Reference LLA ----------
  float lat0_deg;
  float lon0_deg;
  float alt0_m;
  // -------- Debug / monitoring --------
  float last_pos_innov[3]; // ENU (m)
  float last_vel_innov[3]; // ENU (m/s)
};

#endif // ESKF3D_H