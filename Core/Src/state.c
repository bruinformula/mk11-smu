#include "state.h"
#include "gps.h"
#include "can.h"

#include <math.h>

#define DEG_PER_RAD (57.2957795f)
#define GRAVITY_MPS2 (9.80665f)
#define ATTITUDE_COMPLEMENTARY_ALPHA (0.98f)
#define GPS1_HEADING_BLEND_GAIN (0.10f)
#define GPS1_HEADING_MIN_SPEED_MPS (1.0f)
#define GPS1_VEL_KF_Q (0.20f)
#define GPS1_VEL_KF_R (1.50f)

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

volatile float imu_grav_ref_x = 0.0f;
volatile float imu_grav_ref_y = 0.0f;
volatile float imu_grav_ref_z = 1.0f;

volatile float body_slip_angle_deg = 0.0f;

static float imu_cal_R[3][3] = {
	{1.0f, 0.0f, 0.0f},
	{0.0f, 1.0f, 0.0f},
	{0.0f, 0.0f, 1.0f}};

static float imu_cal_sum_gx = 0.0f;
static float imu_cal_sum_gy = 0.0f;
static float imu_cal_sum_gz = 0.0f;
static float imu_cal_sum_ax = 0.0f;
static float imu_cal_sum_ay = 0.0f;
static float imu_cal_sum_az = 0.0f;
static float imu_accel_scale = 1.0f;

static uint32_t last_attitude_update_ms = 0U;
volatile float gps1_vel_kf_state_mps = 0.0f;
volatile float gps1_vel_kf_P = 1.0f;

static void State_ComputeCalRotation(float gx, float gy, float gz)
{
	float mag = sqrtf((gx * gx) + (gy * gy) + (gz * gz));
	if (mag < 1e-6f)
	{
		/* degenerate — leave R as identity */
		return;
	}

	float ux = gx / mag;
	float uy = gy / mag;
	float uz = gz / mag;

	/* cos and sin of the angle between u and [0,0,1] */
	float cos_theta = uz;
	float sin_theta = sqrtf((ux * ux) + (uy * uy));

	if (sin_theta < 1e-6f)
	{
		if (uz > 0.0f)
		{
			/* already pointing +Z — identity */
			imu_cal_R[0][0] = 1.0f;
			imu_cal_R[0][1] = 0.0f;
			imu_cal_R[0][2] = 0.0f;
			imu_cal_R[1][0] = 0.0f;
			imu_cal_R[1][1] = 1.0f;
			imu_cal_R[1][2] = 0.0f;
			imu_cal_R[2][0] = 0.0f;
			imu_cal_R[2][1] = 0.0f;
			imu_cal_R[2][2] = 1.0f;
		}
		else
		{
			/* pointing -Z (upside down) — 180 deg around X */
			imu_cal_R[0][0] = 1.0f;
			imu_cal_R[0][1] = 0.0f;
			imu_cal_R[0][2] = 0.0f;
			imu_cal_R[1][0] = 0.0f;
			imu_cal_R[1][1] = -1.0f;
			imu_cal_R[1][2] = 0.0f;
			imu_cal_R[2][0] = 0.0f;
			imu_cal_R[2][1] = 0.0f;
			imu_cal_R[2][2] = -1.0f;
		}
		return;
	}

	/* rotation axis k = normalize(u x [0,0,1]) = [uy, -ux, 0] / sin_theta */
	float kx = uy / sin_theta;
	float ky = -ux / sin_theta;
	float kz = 0.0f;
	float t = 1.0f - cos_theta;

	/* Rodrigues' rotation formula */
	imu_cal_R[0][0] = cos_theta + (kx * kx * t);
	imu_cal_R[0][1] = (kx * ky * t) - (kz * sin_theta);
	imu_cal_R[0][2] = (kx * kz * t) + (ky * sin_theta);
	imu_cal_R[1][0] = (ky * kx * t) + (kz * sin_theta);
	imu_cal_R[1][1] = cos_theta + (ky * ky * t);
	imu_cal_R[1][2] = (ky * kz * t) - (kx * sin_theta);
	imu_cal_R[2][0] = (kz * kx * t) - (ky * sin_theta);
	imu_cal_R[2][1] = (kz * ky * t) + (kx * sin_theta);
	imu_cal_R[2][2] = cos_theta + (kz * kz * t);
}

static float WrapAngleDeg(float angle_deg)
{
	while (angle_deg >= 180.0f)
	{
		angle_deg -= 360.0f;
	}

	while (angle_deg < -180.0f)
	{
		angle_deg += 360.0f;
	}

	return angle_deg;
}

static float AngleDiffDeg(float target_deg, float source_deg)
{
	return WrapAngleDeg(target_deg - source_deg);
}

static void State_UpdateAttitude(float dt_s)
{
	float accel_roll_deg;
	float accel_pitch_deg;
	float roll_pred_deg;
	float pitch_pred_deg;

	accel_roll_deg = atan2f(imu_ay_corr_g, imu_az_corr_g) * DEG_PER_RAD;
	accel_pitch_deg = atan2f(-imu_ax_corr_g,
							 sqrtf(
								 (imu_ay_corr_g * imu_ay_corr_g) + (imu_az_corr_g * imu_az_corr_g))) *
					  DEG_PER_RAD;

	roll_pred_deg = imu1_roll_deg + (imu_gx_corr_dps * dt_s);
	pitch_pred_deg = imu1_pitch_deg + (imu_gy_corr_dps * dt_s);
	imu1_yaw_deg = WrapAngleDeg(imu1_yaw_deg + (imu_gz_corr_dps * dt_s));

	imu1_roll_deg = (ATTITUDE_COMPLEMENTARY_ALPHA * roll_pred_deg) + ((1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * accel_roll_deg);

	imu1_pitch_deg = (ATTITUDE_COMPLEMENTARY_ALPHA * pitch_pred_deg) + ((1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * accel_pitch_deg);
}

static void State_UpdateGpsNav(float dt_s)
{
	float accel_forward_mps2;
	float gps_speed_mps;
	float K;
	float heading_error_deg;

	gps1_latitude_deg = gps_data.latitude_deg;
	gps1_longitude_deg = gps_data.longitude_deg;
	gps1_altitude_m = gps_data.altitude_m;

	/* Compute forward acceleration in navigation frame by rotating body
	 * accelerations into nav frame, removing gravity, then projecting
	 * onto the forward (body X) axis expressed in nav frame.
	 */
	const float deg2rad = 0.017453292519943295f;
	float roll_rad = imu1_roll_deg * deg2rad;
	float pitch_rad = imu1_pitch_deg * deg2rad;
	float yaw_rad = imu1_yaw_deg * deg2rad;

	float cr = cosf(roll_rad);
	float sr = sinf(roll_rad);
	float cp = cosf(pitch_rad);
	float sp = sinf(pitch_rad);
	float cy = cosf(yaw_rad);
	float sy = sinf(yaw_rad);

	/* Rotation body->nav: R = Rz(yaw) * Ry(pitch) * Rx(roll) */
	float R00 = cy * cp;
	float R01 = cy * sp * sr - sy * cr;
	float R02 = cy * sp * cr + sy * sr;
	float R10 = sy * cp;
	float R11 = sy * sp * sr + cy * cr;
	float R12 = sy * sp * cr - cy * sr;
	float R20 = -sp;
	float R21 = cp * sr;
	float R22 = cp * cr;

	/* body accelerations (m/s^2) */
	float ax_b = imu_ax_corr_g * GRAVITY_MPS2;
	float ay_b = imu_ay_corr_g * GRAVITY_MPS2;
	float az_b = imu_az_corr_g * GRAVITY_MPS2;

	/* nav-frame accelerations */
	float ax_n = (R00 * ax_b) + (R01 * ay_b) + (R02 * az_b);
	float ay_n = (R10 * ax_b) + (R11 * ay_b) + (R12 * az_b);
	float az_n = (R20 * ax_b) + (R21 * ay_b) + (R22 * az_b);

	/* remove gravity (nav Z assumed to include +1g when static) */
	float az_n_lin = az_n - GRAVITY_MPS2;

	/* forward unit vector (body X expressed in nav frame) = first column */
	float ux = R00;
	float uy = R10;
	float uz = R20;

	accel_forward_mps2 = (ax_n * ux) + (ay_n * uy) + (az_n_lin * uz);

	if (gps_data.fix_valid == 1U)
	{
		gps1_vel_kf_state_mps += accel_forward_mps2 * dt_s;
		gps1_vel_kf_P += GPS1_VEL_KF_Q * dt_s;

		gps_speed_mps = gps_data.speed_kph / 3.6f;
		K = gps1_vel_kf_P / (gps1_vel_kf_P + GPS1_VEL_KF_R);
		gps1_vel_kf_state_mps += K * (gps_speed_mps - gps1_vel_kf_state_mps);
		gps1_vel_kf_P *= (1.0f - K);

		gps1_velocity_mps = gps1_vel_kf_state_mps;
	}

	if (gps_data.heading_valid == 1U)
	{
		heading_error_deg = AngleDiffDeg(gps_data.heading_deg, imu1_yaw_deg);
		imu1_yaw_deg = WrapAngleDeg(
			imu1_yaw_deg + (GPS1_HEADING_BLEND_GAIN * heading_error_deg));
	}
	else if ((gps_data.fix_valid == 1U) && (gps1_velocity_mps >= GPS1_HEADING_MIN_SPEED_MPS))
	{
		heading_error_deg = AngleDiffDeg(gps_data.course_deg, imu1_yaw_deg);
		imu1_yaw_deg = WrapAngleDeg(
			imu1_yaw_deg + (GPS1_HEADING_BLEND_GAIN * heading_error_deg));
	}

	if (gps_data.heading_valid == 1U)
	{
		gps1_heading_deg = WrapAngleDeg(gps_data.heading_deg);
	}
	else
	{
		gps1_heading_deg = WrapAngleDeg(imu1_yaw_deg);
	}

	if ((gps_data.fix_valid == 1U) && (gps1_velocity_mps >= GPS1_HEADING_MIN_SPEED_MPS))
	{
		// Calculate difference between vehicle heading (IMU) and the Vehicle course
		body_slip_angle_deg = AngleDiffDeg(imu1_yaw_deg, gps_data.course_deg);
	}
	else
	{
		body_slip_angle_deg = 0.0f;
	}
}


HAL_StatusTypeDef State_Init(void)
{
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

	imu_grav_ref_x = 0.0f;
	imu_grav_ref_y = 0.0f;
	imu_grav_ref_z = 1.0f;

	body_slip_angle_deg = 0.0f;

	imu_cal_R[0][0] = 1.0f;
	imu_cal_R[0][1] = 0.0f;
	imu_cal_R[0][2] = 0.0f;
	imu_cal_R[1][0] = 0.0f;
	imu_cal_R[1][1] = 1.0f;
	imu_cal_R[1][2] = 0.0f;
	imu_cal_R[2][0] = 0.0f;
	imu_cal_R[2][1] = 0.0f;
	imu_cal_R[2][2] = 1.0f;

	imu_cal_sum_gx = 0.0f;
	imu_cal_sum_gy = 0.0f;
	imu_cal_sum_gz = 0.0f;
	imu_cal_sum_ax = 0.0f;
	imu_cal_sum_ay = 0.0f;
	imu_cal_sum_az = 0.0f;
	imu_accel_scale = 1.0f;

	last_attitude_update_ms = 0U;
	gps1_vel_kf_state_mps = 0.0f;
	gps1_vel_kf_P = 1.0f;

	return HAL_OK;
}

void State_UpdateFromImuRaw(int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
							int16_t ax_raw, int16_t ay_raw, int16_t az_raw, uint32_t now_ms)
{
	float dt_s;

	imu_gx_dps = (float)gx_raw * IMU_GYRO_SENS_DPS_PER_LSB;
	imu_gy_dps = (float)gy_raw * IMU_GYRO_SENS_DPS_PER_LSB;
	imu_gz_dps = (float)gz_raw * IMU_GYRO_SENS_DPS_PER_LSB;

	imu_ax_g = (float)ax_raw * IMU_ACCEL_SENS_G_PER_LSB;
	imu_ay_g = (float)ay_raw * IMU_ACCEL_SENS_G_PER_LSB;
	imu_az_g = (float)az_raw * IMU_ACCEL_SENS_G_PER_LSB;

	imu_accel_mag_g = sqrtf(
		(imu_ax_g * imu_ax_g) + (imu_ay_g * imu_ay_g) + (imu_az_g * imu_az_g));

	if (imu_cal_done == 0U)
	{
		imu_cal_sum_gx += imu_gx_dps;
		imu_cal_sum_gy += imu_gy_dps;
		imu_cal_sum_gz += imu_gz_dps;
		imu_cal_sum_ax += imu_ax_g;
		imu_cal_sum_ay += imu_ay_g;
		imu_cal_sum_az += imu_az_g;
		imu_cal_count++;

		if (imu_cal_count >= IMU_CAL_SAMPLES)
		{
			imu_gx_bias_dps = imu_cal_sum_gx / (float)IMU_CAL_SAMPLES;
			imu_gy_bias_dps = imu_cal_sum_gy / (float)IMU_CAL_SAMPLES;
			imu_gz_bias_dps = imu_cal_sum_gz / (float)IMU_CAL_SAMPLES;

			float avg_ax = imu_cal_sum_ax / (float)IMU_CAL_SAMPLES;
			float avg_ay = imu_cal_sum_ay / (float)IMU_CAL_SAMPLES;
			float avg_az = imu_cal_sum_az / (float)IMU_CAL_SAMPLES;

			/* store gravity reference for live inspection */
			imu_grav_ref_x = avg_ax;
			imu_grav_ref_y = avg_ay;
			imu_grav_ref_z = avg_az;

			/* accel biases are zero — rotation matrix handles gravity alignment */
			imu_ax_bias_g = 0.0f;
			imu_ay_bias_g = 0.0f;
			imu_az_bias_g = 0.0f;

			State_ComputeCalRotation(avg_ax, avg_ay, avg_az);

			float grav_mag = sqrtf((avg_ax * avg_ax) + (avg_ay * avg_ay) + (avg_az * avg_az));
			if (grav_mag > 0.5f && grav_mag < 2.0f)
			{
				imu_accel_scale = 1.0f / grav_mag;
			}

			imu_cal_done = 1U;
		}
	}

	imu_gx_corr_dps = imu_gx_dps - imu_gx_bias_dps;
	imu_gy_corr_dps = imu_gy_dps - imu_gy_bias_dps;
	imu_gz_corr_dps = imu_gz_dps - imu_gz_bias_dps;

	imu_ax_corr_g = imu_ax_g - imu_ax_bias_g;
	imu_ay_corr_g = imu_ay_g - imu_ay_bias_g;
	imu_az_corr_g = imu_az_g - imu_az_bias_g;

	/* rotate sensor-frame readings into the calibrated body frame */
	{
		float ax = imu_ax_corr_g;
		float ay = imu_ay_corr_g;
		float az = imu_az_corr_g;
		float gx = imu_gx_corr_dps;
		float gy = imu_gy_corr_dps;
		float gz = imu_gz_corr_dps;

		imu_ax_corr_g = (imu_cal_R[0][0] * ax) + (imu_cal_R[0][1] * ay) + (imu_cal_R[0][2] * az);
		imu_ay_corr_g = (imu_cal_R[1][0] * ax) + (imu_cal_R[1][1] * ay) + (imu_cal_R[1][2] * az);
		imu_az_corr_g = (imu_cal_R[2][0] * ax) + (imu_cal_R[2][1] * ay) + (imu_cal_R[2][2] * az);
		imu_gx_corr_dps = (imu_cal_R[0][0] * gx) + (imu_cal_R[0][1] * gy) + (imu_cal_R[0][2] * gz);
		imu_gy_corr_dps = (imu_cal_R[1][0] * gx) + (imu_cal_R[1][1] * gy) + (imu_cal_R[1][2] * gz);
		imu_gz_corr_dps = (imu_cal_R[2][0] * gx) + (imu_cal_R[2][1] * gy) + (imu_cal_R[2][2] * gz);
	}

	imu_ax_corr_g *= imu_accel_scale;
	imu_ay_corr_g *= imu_accel_scale;
	imu_az_corr_g *= imu_accel_scale;

	if (last_attitude_update_ms == 0U)
	{
		last_attitude_update_ms = now_ms;
		return;
	}

	dt_s = (float)(now_ms - last_attitude_update_ms) * 0.001f;
	last_attitude_update_ms = now_ms;

	if (dt_s < 0.001f)
	{
		dt_s = 0.010f;
	}

	State_UpdateAttitude(dt_s);
#ifdef SMU_GPS_IMU_CROSS
	State_UpdateGpsNav(dt_s);
#endif /* SMU_GPS_IMU_CROSS */
}
