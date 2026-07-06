#ifndef IMU_CONVERT_H
#define IMU_CONVERT_H

#include <stdint.h>

// ======================================================
// IMU Raw → Physical Units Conversion
// Output units are SI (m/s^2, rad/s)
// ======================================================

namespace ImuConvert {

    // ---------- IMU configuration ----------
    enum AccelRange {
        ACCEL_2G,
        ACCEL_4G,
        ACCEL_8G,
        ACCEL_16G
    };

    enum GyroRange {
        GYRO_250DPS,
        GYRO_500DPS,
        GYRO_1000DPS,
        GYRO_2000DPS
    };

    // Must be called once after IMU init
    void configure(
        AccelRange accel_range,
        GyroRange gyro_range
    );

    // ---------- Conversion ----------
    // Raw ADC → m/s^2
    void accelRawToMps2(
        int16_t ax_raw,
        int16_t ay_raw,
        int16_t az_raw,
        float& ax,
        float& ay,
        float& az
    );

    // Raw ADC → rad/s
    void gyroRawToRadps(
        int16_t gx_raw,
        int16_t gy_raw,
        int16_t gz_raw,
        float& gx,
        float& gy,
        float& gz
    );

}

#endif // IMU_CONVERT_H
