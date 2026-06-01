#ifndef ESKF_CONFIG_H
#define ESKF_CONFIG_H
#define ESKF_PI 3.14159265358979323846f

// ======================================================
// ESKF 3D Configuration File
// All constants, noise parameters, and switches live here
// ======================================================

// ---------- Compile-time switches ----------
#define ESKF_USE_GNSS_VELOCITY 1   // 1 = use GNSS velocity update
#define ESKF_USE_ALTITUDE 1        // 1 = fuse altitude
#define ESKF_USE_BIAS_ESTIMATION 1 // 1 = estimate IMU biases
#define ESKF_USE_NHC 1           // 1 = Non-holonomic constraints (no slip/fly)
#define ESKF_USE_GATING 1        // 1 = Outlier rejection (Mahalanobis)
#define ESKF_GATING_THRESH 10.0f // Chi-square threshold (e.g. 3.0 or 5.0)

// ---------- Physical constants ----------
#define ESKF_GRAVITY 9.80665f        // m/s^2
#define ESKF_EARTH_RADIUS 6378137.0f // meters (WGS-84)

// ---------- IMU Noise Parameters ----------
// (continuous-time, 1-sigma)

// Accelerometer noise (m/s^2 / sqrt(Hz))
#define ESKF_SIGMA_ACC 0.15f

// Gyroscope noise (rad/s / sqrt(Hz))
#define ESKF_SIGMA_GYRO                                                        \
  0.005f // Was 0.015. Trust gyro more to catch turns faster.

// Accelerometer bias random walk (m/s^2 / sqrt(Hz))
#define ESKF_SIGMA_ACC_BIAS 0.0005f

// Gyroscope bias random walk (rad/s / sqrt(Hz))
#define ESKF_SIGMA_GYRO_BIAS 0.0002f

// ---------- GNSS Measurement Noise ----------

// Position noise (meters, 1-sigma)
#define ESKF_SIGMA_GNSS_POS 3.0f // Was 1.5. Trust GPS less (it has lag).

// Velocity noise (m/s, 1-sigma)
#define ESKF_SIGMA_GNSS_VEL 0.15f

// Altitude noise (meters, 1-sigma)
#define ESKF_SIGMA_GNSS_ALT 2.5f

// ---------- Numerical limits ----------
#define ESKF_MAX_DT 0.2f // seconds (reject IMU gaps beyond this)
#define ESKF_MIN_DT 0.0005f

// ---------- ZUPT Thresholds ----------
#define ESKF_ZUPT_ACC_STD 0.3f  // m/s^2 (std deviation to consider static)
#define ESKF_ZUPT_GYR_STD 0.05f // rad/s

#endif // ESKF_CONFIG_H
