#include "ImuConvert.h"
#include "EskfConfig.h"
#include <math.h>

namespace ImuConvert {

    // ---------- internal scale factors ----------
    static float acc_scale = 0.0f;   // m/s^2 per LSB
    static float gyr_scale = 0.0f;   // rad/s per LSB

    void configure(AccelRange accel_range,
                   GyroRange gyro_range)
    {
        // ---- Accelerometer ----
        switch (accel_range) {
            case ACCEL_2G:  acc_scale = (2.0f * ESKF_GRAVITY) / 32768.0f; break;
            case ACCEL_4G:  acc_scale = (4.0f * ESKF_GRAVITY) / 32768.0f; break;
            case ACCEL_8G:  acc_scale = (8.0f * ESKF_GRAVITY) / 32768.0f; break;
            case ACCEL_16G: acc_scale = (16.0f * ESKF_GRAVITY) / 32768.0f; break;
        }

        // ---- Gyroscope ----
        switch (gyro_range) {
            case GYRO_250DPS:  gyr_scale = (250.0f * (float)ESKF_PI / 180.0f) / 32768.0f; break;
            case GYRO_500DPS:  gyr_scale = (500.0f * (float)ESKF_PI / 180.0f) / 32768.0f; break;
            case GYRO_1000DPS: gyr_scale = (1000.0f * (float)ESKF_PI / 180.0f) / 32768.0f; break;
            case GYRO_2000DPS: gyr_scale = (2000.0f * (float)ESKF_PI / 180.0f) / 32768.0f; break;
        }
    }

    void accelRawToMps2(int16_t ax_raw,
                        int16_t ay_raw,
                        int16_t az_raw,
                        float& ax,
                        float& ay,
                        float& az)
    {
        ax = ax_raw * acc_scale;
        ay = ay_raw * acc_scale;
        az = -az_raw * acc_scale;
    }

    void gyroRawToRadps(int16_t gx_raw,
                        int16_t gy_raw,
                        int16_t gz_raw,
                        float& gx,
                        float& gy,
                        float& gz)
    {
        gx = gx_raw * gyr_scale;
        gy = gy_raw * gyr_scale;
        gz = -gz_raw * gyr_scale;
    }

} // namespace ImuConvert
