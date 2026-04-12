#include "state.h"
#include "gps.h"

#include <math.h>

#define DEG_PER_RAD                  (57.2957795f)
#define GRAVITY_MPS2                 (9.80665f)
#define ATTITUDE_COMPLEMENTARY_ALPHA (0.98f)
#define GPS1_HEADING_BLEND_GAIN      (0.10f)
#define GPS1_HEADING_MIN_SPEED_MPS   (1.0f)
#define GPS1_VEL_KF_Q                (0.20f)
#define GPS1_VEL_KF_R                (1.50f)

volatile uint8_t imu_cal_done = 0U;
volatile uint32_t imu_cal_count = 0U;

volatile float imu_gx_dps = 0.0f;
volatile float imu_gy_dps = 0.0f;
volatile float imu_gz_dps = 0.0f;
volatile float imu_ax_g = 0.0f;
volatile float imu_ay_g = 0.0f;
volatile float imu_az_g = 0.0f;

volatile float imu_gx_bias_dps = 0.0f;
volatile float imu_gy_bias_dps = 0.0f;
volatile float imu_gz_bias_dps = 0.0f;
volatile float imu_ax_bias_g = 0.0f;
volatile float imu_ay_bias_g = 0.0f;
volatile float imu_az_bias_g = 0.0f;

volatile float imu_gx_corr_dps = 0.0f;
volatile float imu_gy_corr_dps = 0.0f;
volatile float imu_gz_corr_dps = 0.0f;
volatile float imu_ax_corr_g = 0.0f;
volatile float imu_ay_corr_g = 0.0f;
volatile float imu_az_corr_g = 0.0f;
volatile float imu_accel_mag_g = 0.0f;

volatile float gps1_latitude_deg = 0.0f;
volatile float gps1_longitude_deg = 0.0f;
volatile float gps1_altitude_m = 0.0f;
volatile float gps1_velocity_mps = 0.0f;
volatile float gps1_heading_deg = 0.0f;

volatile float imu1_pitch_deg = 0.0f;
volatile float imu1_roll_deg = 0.0f;
volatile float imu1_yaw_deg = 0.0f;

static float imu_cal_sum_gx = 0.0f;
static float imu_cal_sum_gy = 0.0f;
static float imu_cal_sum_gz = 0.0f;
static float imu_cal_sum_ax = 0.0f;
static float imu_cal_sum_ay = 0.0f;
static float imu_cal_sum_az = 0.0f;

static uint32_t last_attitude_update_ms = 0U;
static float gps1_vel_kf_state_mps = 0.0f;
static float gps1_vel_kf_P = 1.0f;

static float WrapAngleDeg(float angle_deg) {
	while (angle_deg >= 180.0f) {
		angle_deg -= 360.0f;
	}

	while (angle_deg < -180.0f) {
		angle_deg += 360.0f;
	}

	return angle_deg;
}

static float AngleDiffDeg(float target_deg, float source_deg) {
	return WrapAngleDeg(target_deg - source_deg);
}

static void State_UpdateAttitude(float dt_s) {
	float accel_roll_deg;
	float accel_pitch_deg;
	float roll_pred_deg;
	float pitch_pred_deg;

	accel_roll_deg = atan2f(imu_ay_corr_g, imu_az_corr_g) * DEG_PER_RAD;
	accel_pitch_deg = atan2f(-imu_ax_corr_g,
			sqrtf(
					(imu_ay_corr_g * imu_ay_corr_g)
							+ (imu_az_corr_g * imu_az_corr_g))) * DEG_PER_RAD;

	roll_pred_deg = imu1_roll_deg + (imu_gx_corr_dps * dt_s);
	pitch_pred_deg = imu1_pitch_deg + (imu_gy_corr_dps * dt_s);
	imu1_yaw_deg = WrapAngleDeg(imu1_yaw_deg + (imu_gz_corr_dps * dt_s));

	imu1_roll_deg = (ATTITUDE_COMPLEMENTARY_ALPHA * roll_pred_deg)
			+ ((1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * accel_roll_deg);

	imu1_pitch_deg = (ATTITUDE_COMPLEMENTARY_ALPHA * pitch_pred_deg)
			+ ((1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * accel_pitch_deg);
}

static void State_UpdateGpsNav(float dt_s) {
	float accel_forward_mps2;
	float gps_speed_mps;
	float K;
	float heading_error_deg;

	gps1_latitude_deg = gps_data.latitude_deg;
	gps1_longitude_deg = gps_data.longitude_deg;
	gps1_altitude_m = gps_data.altitude_m;

	accel_forward_mps2 = imu_ax_corr_g * GRAVITY_MPS2;

	gps1_vel_kf_state_mps += accel_forward_mps2 * dt_s;
	gps1_vel_kf_P += GPS1_VEL_KF_Q * dt_s;

	gps_speed_mps = gps_data.speed_kph / 3.6f;
	K = gps1_vel_kf_P / (gps1_vel_kf_P + GPS1_VEL_KF_R);
	gps1_vel_kf_state_mps += K * (gps_speed_mps - gps1_vel_kf_state_mps);
	gps1_vel_kf_P *= (1.0f - K);

	gps1_velocity_mps = gps1_vel_kf_state_mps;

	if ((gps_data.fix_valid == 1U)
			&& (gps1_velocity_mps >= GPS1_HEADING_MIN_SPEED_MPS)) {
		heading_error_deg = AngleDiffDeg(gps_data.course_deg, imu1_yaw_deg);
		imu1_yaw_deg = WrapAngleDeg(
				imu1_yaw_deg + (GPS1_HEADING_BLEND_GAIN * heading_error_deg));
	}

	gps1_heading_deg = WrapAngleDeg(imu1_yaw_deg);
}

HAL_StatusTypeDef State_Init(void) {
	imu_cal_done = 0U;
	imu_cal_count = 0U;

	imu_gx_dps = 0.0f;
	imu_gy_dps = 0.0f;
	imu_gz_dps = 0.0f;
	imu_ax_g = 0.0f;
	imu_ay_g = 0.0f;
	imu_az_g = 0.0f;

	imu_gx_bias_dps = 0.0f;
	imu_gy_bias_dps = 0.0f;
	imu_gz_bias_dps = 0.0f;
	imu_ax_bias_g = 0.0f;
	imu_ay_bias_g = 0.0f;
	imu_az_bias_g = 0.0f;

	imu_gx_corr_dps = 0.0f;
	imu_gy_corr_dps = 0.0f;
	imu_gz_corr_dps = 0.0f;
	imu_ax_corr_g = 0.0f;
	imu_ay_corr_g = 0.0f;
	imu_az_corr_g = 0.0f;
	imu_accel_mag_g = 0.0f;

	gps1_latitude_deg = 0.0f;
	gps1_longitude_deg = 0.0f;
	gps1_altitude_m = 0.0f;
	gps1_velocity_mps = 0.0f;
	gps1_heading_deg = 0.0f;

	imu1_pitch_deg = 0.0f;
	imu1_roll_deg = 0.0f;
	imu1_yaw_deg = 0.0f;

	imu_cal_sum_gx = 0.0f;
	imu_cal_sum_gy = 0.0f;
	imu_cal_sum_gz = 0.0f;
	imu_cal_sum_ax = 0.0f;
	imu_cal_sum_ay = 0.0f;
	imu_cal_sum_az = 0.0f;

	last_attitude_update_ms = 0U;
	gps1_vel_kf_state_mps = 0.0f;
	gps1_vel_kf_P = 1.0f;

	return HAL_OK;
}

void State_UpdateFromImuRaw(int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
		int16_t ax_raw, int16_t ay_raw, int16_t az_raw, uint32_t now_ms) {
	float dt_s;

	imu_gx_dps = (float) gx_raw * IMU_GYRO_SENS_DPS_PER_LSB;
	imu_gy_dps = (float) gy_raw * IMU_GYRO_SENS_DPS_PER_LSB;
	imu_gz_dps = (float) gz_raw * IMU_GYRO_SENS_DPS_PER_LSB;

	imu_ax_g = (float) ax_raw * IMU_ACCEL_SENS_G_PER_LSB;
	imu_ay_g = (float) ay_raw * IMU_ACCEL_SENS_G_PER_LSB;
	imu_az_g = (float) az_raw * IMU_ACCEL_SENS_G_PER_LSB;

	imu_accel_mag_g = sqrtf(
			(imu_ax_g * imu_ax_g) + (imu_ay_g * imu_ay_g)
					+ (imu_az_g * imu_az_g));

	if (imu_cal_done == 0U) {
		imu_cal_sum_gx += imu_gx_dps;
		imu_cal_sum_gy += imu_gy_dps;
		imu_cal_sum_gz += imu_gz_dps;
		imu_cal_sum_ax += imu_ax_g;
		imu_cal_sum_ay += imu_ay_g;
		imu_cal_sum_az += imu_az_g;
		imu_cal_count++;

		if (imu_cal_count >= IMU_CAL_SAMPLES) {
			imu_gx_bias_dps = imu_cal_sum_gx / (float) IMU_CAL_SAMPLES;
			imu_gy_bias_dps = imu_cal_sum_gy / (float) IMU_CAL_SAMPLES;
			imu_gz_bias_dps = imu_cal_sum_gz / (float) IMU_CAL_SAMPLES;

			imu_ax_bias_g = imu_cal_sum_ax / (float) IMU_CAL_SAMPLES;
			imu_ay_bias_g = imu_cal_sum_ay / (float) IMU_CAL_SAMPLES;
			imu_az_bias_g = (imu_cal_sum_az / (float) IMU_CAL_SAMPLES) - 1.0f;

			imu_cal_done = 1U;
		}
	}

	imu_gx_corr_dps = imu_gx_dps - imu_gx_bias_dps;
	imu_gy_corr_dps = imu_gy_dps - imu_gy_bias_dps;
	imu_gz_corr_dps = imu_gz_dps - imu_gz_bias_dps;

	imu_ax_corr_g = imu_ax_g - imu_ax_bias_g;
	imu_ay_corr_g = imu_ay_g - imu_ay_bias_g;
	imu_az_corr_g = imu_az_g - imu_az_bias_g;

	if (last_attitude_update_ms == 0U) {
		last_attitude_update_ms = now_ms;
		return;
	}

	dt_s = (float) (now_ms - last_attitude_update_ms) * 0.001f;
	last_attitude_update_ms = now_ms;

	if (dt_s < 0.001f) {
		dt_s = 0.010f;
	}

	State_UpdateAttitude(dt_s);
	State_UpdateGpsNav(dt_s);
}
