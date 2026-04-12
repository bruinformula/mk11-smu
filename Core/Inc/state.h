#ifndef STATE_H
#define STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define IMU_ACCEL_SENS_G_PER_LSB     (0.000061f)
#define IMU_GYRO_SENS_DPS_PER_LSB    (0.00875f)
#define IMU_CAL_SAMPLES              200U

extern volatile uint8_t imu_cal_done;
extern volatile uint32_t imu_cal_count;

extern volatile float imu_gx_dps;
extern volatile float imu_gy_dps;
extern volatile float imu_gz_dps;
extern volatile float imu_ax_g;
extern volatile float imu_ay_g;
extern volatile float imu_az_g;

extern volatile float imu_gx_bias_dps;
extern volatile float imu_gy_bias_dps;
extern volatile float imu_gz_bias_dps;
extern volatile float imu_ax_bias_g;
extern volatile float imu_ay_bias_g;
extern volatile float imu_az_bias_g;

extern volatile float imu_gx_corr_dps;
extern volatile float imu_gy_corr_dps;
extern volatile float imu_gz_corr_dps;
extern volatile float imu_ax_corr_g;
extern volatile float imu_ay_corr_g;
extern volatile float imu_az_corr_g;
extern volatile float imu_accel_mag_g;

extern volatile float gps1_latitude_deg;
extern volatile float gps1_longitude_deg;
extern volatile float gps1_altitude_m;
extern volatile float gps1_velocity_mps;
extern volatile float gps1_heading_deg;

extern volatile float imu1_pitch_deg;
extern volatile float imu1_roll_deg;
extern volatile float imu1_yaw_deg;

HAL_StatusTypeDef State_Init(void);
void State_UpdateFromImuRaw(int16_t gx_raw,
                            int16_t gy_raw,
                            int16_t gz_raw,
                            int16_t ax_raw,
                            int16_t ay_raw,
                            int16_t az_raw,
                            uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* STATE_H */
