#include "can.h"
#include "state.h"
#include "gps.h"

#include <string.h>
#include <math.h>

#define IMU1_ACCEL_TX_INTERVAL_MS 10U
#define IMU1_ATT_TX_INTERVAL_MS 10U
#define GPS_COG_TIMESYNC_TX_INTERVAL_MS 100U
#define GPS_COG_DATA_TX_INTERVAL_MS 50U
#define CAN_GPS_COG_FRAME_BYTES 64U
#define CAN_GPS_ERROR_NO_FIX 0x01U
#define CAN_GPS_ERROR_UART 0x02U
#define CAN_GPS_ERROR_NO_SENTENCES 0x04U

#define IMU_FD_TX_INTERVAL_MS 10U
#define IMU_FD_EXPECTED_PERIOD 100U /* 100-us units → 10 ms between samples */
#define IMU_FD_DT_S 0.010f
#define IMU_FD_GRAVITY_MPS2 9.80665f

volatile uint32_t fdcan_tx_count = 0U;
volatile uint32_t fdcan_rx_count = 0U;
volatile uint32_t fdcan_rx_error_count = 0U;
volatile uint32_t fdcan1_debug_cb = 0U;

static FDCAN_HandleTypeDef *can_fdcan = NULL;
static FDCAN_TxHeaderTypeDef can_tx_header;
static uint8_t can_tx_data[CAN_GPS_COG_FRAME_BYTES];

static uint32_t last_imu1_accel_tx_time = 0U;
static uint32_t last_imu1_att_tx_time = 0U;
static uint32_t last_gps1_fd_tx_time = 0U;
static uint32_t last_gps_cog_data_tx_time = 0U;

static uint32_t last_imu_fd_tx_time = 0U;
static uint8_t imu_fd_half = 0U;		/* 0=capture s1, 1=capture s2+send */
static uint8_t imu_fd_msg_counter = 0U; /* 3-bit rolling message counter */

/* Sample-1 snapshot fields */
static int16_t imu_fd_s1_ax, imu_fd_s1_ay, imu_fd_s1_az;
static int16_t imu_fd_s1_aa, imu_fd_s1_ab, imu_fd_s1_ac;
static int16_t imu_fd_s1_vx, imu_fd_s1_vy, imu_fd_s1_vz;
static int16_t imu_fd_s1_gx, imu_fd_s1_gy, imu_fd_s1_gz;
static uint32_t imu_fd_s1_ts_us;

/* Running integration / differentiation state */
static float imu_fd_prev_gx_dps = 0.0f;
static float imu_fd_prev_gy_dps = 0.0f;
static float imu_fd_prev_gz_dps = 0.0f;
static float imu_fd_vel_x_mps = 0.0f;
static float imu_fd_vel_y_mps = 0.0f;
static float imu_fd_vel_z_mps = 0.0f;

/* SDU Wheel Speed Parsing and Fusion variables */
static volatile float sdu_wheel_rpm[4] = {0.0f};
static volatile uint32_t last_sdu_rx_time[4] = {0};
static volatile float average_wheel_speed_mps = 0.0f;
static volatile uint32_t last_sdu_can_rx_time = 0;

/* Accelerometer Variance ZUPT (Mode 3) variables */
#define ZUPT_WINDOW_SIZE 10
#define ZUPT_VAR_THRESHOLD 0.002f // Variance threshold in g^2

/* Safety thresholds and debounce for IMU-only ZUPT */
#define ZUPT_CONFIRM_MS 500U
#define ZUPT_ACCEL_MAG_TOL 0.05f
#define ZUPT_GYRO_DPS_THR 5.0f

static float accel_mag_history[ZUPT_WINDOW_SIZE] = {
	1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
static uint8_t zupt_index = 0;

/* ZUPT debounce / diagnostics state */
static uint32_t zupt_candidate_start_ms = 0U;
static uint8_t zupt_confirmed = 0U;
static uint32_t zupt_trigger_count = 0U;
static uint32_t zupt_blocked_no_external = 0U;
/* If set to 1, allow IMU-only ZUPT when GPS/SDU are absent (default 0)
 * Can be exposed via config if desired.
 */
static uint8_t allow_zupt_without_external = 0U;

static uint8_t can_imu_comm_ok = 0U;
static uint8_t can_imu_init_ok = 0U;

static int16_t CAN_ClampS16(float value)
{
	if (value > 32767.0f)
	{
		return 32767;
	}

	if (value < -32768.0f)
	{
		return -32768;
	}

	return (int16_t)value;
}

static uint16_t CAN_ClampU16(float value)
{
	if (value > 65535.0f)
	{
		return 65535U;
	}

	if (value < 0.0f)
	{
		return 0U;
	}

	return (uint16_t)value;
}

static uint32_t CAN_ClampU32(float value)
{
	if (value > 4294967295.0f)
	{
		return 4294967295UL;
	}

	if (value < 0.0f)
	{
		return 0U;
	}

	return (uint32_t)value;
}

static int32_t CAN_ClampS32(float value)
{
	if (value > 2147483647.0f)
	{
		return 2147483647;
	}

	if (value < -2147483648.0f)
	{
		return (-2147483647 - 1);
	}

	return (int32_t)value;
}

static void CAN_PackS16LE(uint8_t *data, uint8_t idx, int16_t value)
{
	data[idx] = (uint8_t)(value & 0xFF);
	data[idx + 1U] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
}

static void CAN_PackU16LE(uint8_t *data, uint8_t idx, uint16_t value)
{
	data[idx] = (uint8_t)(value & 0xFFU);
	data[idx + 1U] = (uint8_t)((value >> 8) & 0xFFU);
}

static void CAN_PackS32LE(uint8_t *data, uint8_t idx, int32_t value)
{
	data[idx] = (uint8_t)(value & 0xFF);
	data[idx + 1U] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
	data[idx + 2U] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
	data[idx + 3U] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
}

static void CAN_PackU32LE(uint8_t *data, uint8_t idx, uint32_t value)
{
	data[idx] = (uint8_t)(value & 0xFFU);
	data[idx + 1U] = (uint8_t)((value >> 8) & 0xFFU);
	data[idx + 2U] = (uint8_t)((value >> 16) & 0xFFU);
	data[idx + 3U] = (uint8_t)((value >> 24) & 0xFFU);
}

static void CAN_PackFloatLE(uint8_t *data, uint8_t idx, float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	CAN_PackU32LE(data, idx, bits);
}


static uint16_t CAN_GpsErrorFlags(void)
{
	uint16_t flags = 0U;

	if (gps_data.fix_valid == 0U)
	{
		flags |= CAN_GPS_ERROR_NO_FIX;
	}

	if (gps_diag.uart_last_error_code != 0U)
	{
		flags |= CAN_GPS_ERROR_UART;
	}

	if (gps_diag.sentence_count == 0U)
	{
		flags |= CAN_GPS_ERROR_NO_SENTENCES;
	}

	return flags;
}

static HAL_StatusTypeDef CAN_Send(uint32_t id, uint32_t data_length,
								  uint32_t fd_format)
{
	if (can_fdcan == NULL)
	{
		return HAL_ERROR;
	}

	can_tx_header.Identifier = id;
	can_tx_header.DataLength = data_length;
	can_tx_header.FDFormat = fd_format;
	can_tx_header.BitRateSwitch = (fd_format == FDCAN_FD_CAN) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;

	if (HAL_FDCAN_AddMessageToTxFifoQ(can_fdcan, &can_tx_header, can_tx_data) == HAL_OK)
	{
		fdcan_tx_count++;
		return HAL_OK;
	}

	return HAL_ERROR;
}

static void CAN_SendImuAccel(void)
{
	int16_t ax_mg;
	int16_t ay_mg;
	int16_t az_mg;

	ax_mg = CAN_ClampS16(imu_ax_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);
	ay_mg = CAN_ClampS16(imu_ay_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);
	az_mg = CAN_ClampS16(imu_az_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);

	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackS16LE(can_tx_data, 0U, ax_mg);
	CAN_PackS16LE(can_tx_data, 2U, ay_mg);
	CAN_PackS16LE(can_tx_data, 4U, az_mg);
	can_tx_data[6] = imu_cal_done;
	can_tx_data[7] = 0U;

	(void)CAN_Send(IMU1_ACCEL_TX_ID, FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN);
}

static void CAN_SendImuAtt(void)
{
	int16_t pitch_cdeg;
	int16_t roll_cdeg;
	int16_t yaw_cdeg;

	pitch_cdeg = CAN_ClampS16(imu1_pitch_deg * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	roll_cdeg = CAN_ClampS16(imu1_roll_deg * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	yaw_cdeg = CAN_ClampS16(imu1_yaw_deg * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);

	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackS16LE(can_tx_data, 0U, pitch_cdeg);
	CAN_PackS16LE(can_tx_data, 2U, roll_cdeg);
	CAN_PackS16LE(can_tx_data, 4U, yaw_cdeg);
	can_tx_data[6] = can_imu_comm_ok;
	can_tx_data[7] = can_imu_init_ok;

	(void)CAN_Send(IMU1_ATT_TX_ID, FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN);
}

static void CAN_SendGpsFd(uint32_t now_ms)
{
	uint32_t age_ms;
	uint32_t us_since_last;
	uint16_t length_cx100;
	uint16_t heading_acc_cx100;
	uint16_t pitch_acc_cx100;

	age_ms = now_ms - gps_data.last_update_ms;
	us_since_last = (age_ms <= 4294967U) ? (age_ms * 1000U) : 0xFFFFFFFFU;

	length_cx100 = CAN_ClampU16(gps_data.baseline_length_m * GPS1_LENGTH_CAN_SCALE_CX100_PER_M);
	heading_acc_cx100 = CAN_ClampU16(gps_data.heading_accuracy_deg * GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG);
	pitch_acc_cx100 = CAN_ClampU16(gps_data.pitch_acc_deg * GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG);

	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackU32LE(can_tx_data, 0U, us_since_last);                    /* bytes 0-3  */
	CAN_PackU16LE(can_tx_data, 4U, CAN_GpsErrorFlags());              /* bytes 4-5  */
	CAN_PackFloatLE(can_tx_data, 6U, gps_data.latitude_deg);          /* bytes 6-9  */
	CAN_PackFloatLE(can_tx_data, 10U, gps_data.longitude_deg);        /* bytes 10-13 */
	CAN_PackFloatLE(can_tx_data, 14U, gps1_velocity_mps);             /* bytes 14-17 */
	can_tx_data[18] = gps_data.fix_quality;                           /* byte 18    */
	CAN_PackU16LE(can_tx_data, 19U, length_cx100);                    /* bytes 19-20 */
	CAN_PackFloatLE(can_tx_data, 21U, gps_data.heading_deg);          /* bytes 21-24 */
	CAN_PackU16LE(can_tx_data, 25U, heading_acc_cx100);               /* bytes 25-26 */
	CAN_PackFloatLE(can_tx_data, 27U, gps_data.pitch_deg);            /* bytes 27-30 */
	CAN_PackU16LE(can_tx_data, 31U, pitch_acc_cx100);                 /* bytes 31-32 */
	can_tx_data[33] = gps_data.tar_satellites;                        /* byte 33    */
	/* bytes 34-63 reserved, zeroed */

	(void)CAN_Send(GPS_COG_TIMESYNC_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

static void CAN_SendGpsCogPos(uint32_t now_ms)
{
	int32_t lat_dege7;
	int32_t lon_dege7;
	int32_t alt_mm;
	uint16_t hdop_centi;

	lat_dege7 = CAN_ClampS32(
		gps1_latitude_deg * GPS1_POS_CAN_SCALE_DEGE7_PER_DEG);
	lon_dege7 = CAN_ClampS32(
		gps1_longitude_deg * GPS1_POS_CAN_SCALE_DEGE7_PER_DEG);
	alt_mm = CAN_ClampS32(gps1_altitude_m * GPS1_ALT_CAN_SCALE_MM_PER_M);
	hdop_centi = CAN_ClampU16(gps_data.hdop * GPS1_HDOP_CAN_SCALE_CENTI_PER_UNIT);

	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackU32LE(can_tx_data, 0U, now_ms * 1000U);
	CAN_PackS32LE(can_tx_data, 4U, lat_dege7);
	CAN_PackS32LE(can_tx_data, 8U, lon_dege7);
	CAN_PackS32LE(can_tx_data, 12U, alt_mm);
	CAN_PackU16LE(can_tx_data, 16U, hdop_centi);
	can_tx_data[18] = gps_data.fix_valid;
	can_tx_data[19] = gps_data.fix_quality;
	can_tx_data[20] = gps_data.satellites;
	can_tx_data[63] = CAN_GpsErrorFlags();

	(void)CAN_Send(GPS_COG_POS_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

static void CAN_SendGpsCogNav(uint32_t now_ms)
{
	uint32_t vel_cmps;
	int32_t course_cdeg;
	int32_t heading_cdeg;
	uint16_t heading_accuracy_cdeg;
	uint32_t baseline_mm;
	int32_t pitch_cdeg;

	vel_cmps = CAN_ClampU32(
		gps1_velocity_mps * GPS1_VEL_CAN_SCALE_CMPS_PER_MPS);
	course_cdeg = CAN_ClampS32(
		gps_data.course_deg * GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG);
	heading_cdeg = CAN_ClampS32(
		gps1_heading_deg * GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG);
	heading_accuracy_cdeg = CAN_ClampU16(
		gps_data.heading_accuracy_deg * GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG);
	baseline_mm = CAN_ClampU32(
		gps_data.baseline_length_m * GPS1_BASELINE_CAN_SCALE_MM_PER_M);
	pitch_cdeg = CAN_ClampS32(gps_data.pitch_deg * GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG);

	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackU32LE(can_tx_data, 0U, now_ms * 1000U);
	CAN_PackU32LE(can_tx_data, 4U, vel_cmps);
	CAN_PackS32LE(can_tx_data, 8U, course_cdeg);
	CAN_PackS32LE(can_tx_data, 12U, heading_cdeg);
	CAN_PackU16LE(can_tx_data, 16U, heading_accuracy_cdeg);
	can_tx_data[18] = gps_data.heading_valid;
	can_tx_data[19] = gps_data.heading_quality;
	CAN_PackU32LE(can_tx_data, 20U, baseline_mm);
	CAN_PackS32LE(can_tx_data, 24U, pitch_cdeg);
	can_tx_data[63] = CAN_GpsErrorFlags();

	(void)CAN_Send(GPS_COG_NAV_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

static void CAN_SendGpsCogImu(uint32_t now_ms)
{
	int16_t ax_mg;
	int16_t ay_mg;
	int16_t az_mg;
	int16_t gx_cdps;
	int16_t gy_cdps;
	int16_t gz_cdps;
	int16_t pitch_cdeg;
	int16_t roll_cdeg;
	int16_t yaw_cdeg;

	ax_mg = CAN_ClampS16(imu_ax_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);
	ay_mg = CAN_ClampS16(imu_ay_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);
	az_mg = CAN_ClampS16(imu_az_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);
	gx_cdps = CAN_ClampS16(imu_gx_corr_dps * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	gy_cdps = CAN_ClampS16(imu_gy_corr_dps * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	gz_cdps = CAN_ClampS16(imu_gz_corr_dps * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	pitch_cdeg = CAN_ClampS16(imu1_pitch_deg * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	roll_cdeg = CAN_ClampS16(imu1_roll_deg * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	yaw_cdeg = CAN_ClampS16(imu1_yaw_deg * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);

	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackU32LE(can_tx_data, 0U, now_ms * 1000U);
	CAN_PackS16LE(can_tx_data, 4U, ax_mg);
	CAN_PackS16LE(can_tx_data, 6U, ay_mg);
	CAN_PackS16LE(can_tx_data, 8U, az_mg);
	CAN_PackS16LE(can_tx_data, 10U, gx_cdps);
	CAN_PackS16LE(can_tx_data, 12U, gy_cdps);
	CAN_PackS16LE(can_tx_data, 14U, gz_cdps);
	CAN_PackS16LE(can_tx_data, 16U, pitch_cdeg);
	CAN_PackS16LE(can_tx_data, 18U, roll_cdeg);
	CAN_PackS16LE(can_tx_data, 20U, yaw_cdeg);
	can_tx_data[22] = imu_cal_done;
	can_tx_data[23] = can_imu_comm_ok;
	can_tx_data[24] = can_imu_init_ok;
	can_tx_data[63] = (uint8_t)((can_imu_comm_ok == 0U) ? 0x01U : 0U);

	(void)CAN_Send(GPS_COG_IMU_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

static uint8_t DetectStationaryByVariance(float new_accel_mag)
{
	accel_mag_history[zupt_index] = new_accel_mag;
	zupt_index = (zupt_index + 1) % ZUPT_WINDOW_SIZE;

	// Calculate mean
	float sum = 0.0f;
	for (int i = 0; i < ZUPT_WINDOW_SIZE; i++)
	{
		sum += accel_mag_history[i];
	}
	float mean = sum / ZUPT_WINDOW_SIZE;

	// Calculate variance
	float var_sum = 0.0f;
	for (int i = 0; i < ZUPT_WINDOW_SIZE; i++)
	{
		float diff = accel_mag_history[i] - mean;
		var_sum += diff * diff;
	}
	float variance = var_sum / ZUPT_WINDOW_SIZE;

	// If variance is extremely low, the sensor is stationary
	return (variance < ZUPT_VAR_THRESHOLD);
}

static void CAN_UpdateVelocity(float dt_s, uint32_t now_ms)
{
	extern volatile float gps1_velocity_mps;
	extern volatile float gps1_vel_kf_state_mps;
	extern volatile float gps1_vel_kf_P;

	uint8_t gps_locked = gps_data.fix_valid;
	uint8_t sdu_speeds_available = (now_ms - last_sdu_can_rx_time) < 500; // 500ms timeout
	uint8_t is_stationary = 0;

	/* Calculate SDU average wheel speed if available */
	float rpm_sum = 0.0f;
	int active_count = 0;
	for (int i = 0; i < 4; i++)
	{
		if ((now_ms - last_sdu_rx_time[i]) < 500)
		{
			rpm_sum += sdu_wheel_rpm[i];
			active_count++;
		}
	}

	if (active_count > 0)
	{
		float average_wheel_rpm = rpm_sum / (float)active_count;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define SDU_TIRE_DIAMETER_INCHES 18.0f
#define SDU_TIRE_CIRCUMFERENCE_M (SDU_TIRE_DIAMETER_INCHES * 0.0254f * M_PI)

		average_wheel_speed_mps = (average_wheel_rpm / 60.0f) * SDU_TIRE_CIRCUMFERENCE_M;
		last_sdu_can_rx_time = now_ms;
	}

	/* Determine stationary status (ZUPT) with safer rules */
	if (gps_locked)
	{
		is_stationary = (gps_data.speed_kph < 0.5f);
		zupt_candidate_start_ms = 0U;
		zupt_confirmed = 0U;
	}
	else if (sdu_speeds_available)
	{
		is_stationary = (average_wheel_speed_mps < 0.1f);
		zupt_candidate_start_ms = 0U;
		zupt_confirmed = 0U;
	}
	else
	{
		uint8_t imu_stationary_vote = 0U;
		if (imu_cal_done)
		{
			float accel_mag_delta = imu_accel_mag_g - 1.0f;
			if (accel_mag_delta < 0.0f)
				accel_mag_delta = -accel_mag_delta;

			float gx_abs = imu_gx_corr_dps < 0.0f ? -imu_gx_corr_dps : imu_gx_corr_dps;
			float gy_abs = imu_gy_corr_dps < 0.0f ? -imu_gy_corr_dps : imu_gy_corr_dps;
			float gz_abs = imu_gz_corr_dps < 0.0f ? -imu_gz_corr_dps : imu_gz_corr_dps;

			if ((accel_mag_delta <= ZUPT_ACCEL_MAG_TOL) && (gx_abs <= ZUPT_GYRO_DPS_THR) && (gy_abs <= ZUPT_GYRO_DPS_THR) && (gz_abs <= ZUPT_GYRO_DPS_THR))
			{
				if (DetectStationaryByVariance(imu_accel_mag_g))
				{
					imu_stationary_vote = 1U;
				}
			}
		}

		if (imu_stationary_vote)
		{
			if (zupt_candidate_start_ms == 0U)
			{
				zupt_candidate_start_ms = now_ms;
				zupt_confirmed = 0U;
			}
			else if ((now_ms - zupt_candidate_start_ms) >= ZUPT_CONFIRM_MS)
			{
				zupt_confirmed = 1U;
			}

			if (zupt_confirmed && allow_zupt_without_external)
			{
				is_stationary = 1U;
			}
			else
			{
				if (zupt_confirmed)
					zupt_blocked_no_external++;
				is_stationary = 0U;
			}
		}
		else
		{
			zupt_candidate_start_ms = 0U;
			zupt_confirmed = 0U;
			is_stationary = 0U;
		}
	}

	if (is_stationary)
	{
		uint8_t gps_present = (gps_diag.sentence_count != 0U);
		uint8_t sdu_present = (active_count > 0);

		imu_fd_vel_x_mps = 0.0f;
		imu_fd_vel_y_mps = 0.0f;
		imu_fd_vel_z_mps = 0.0f;

		if (gps_present || sdu_present || allow_zupt_without_external)
		{
			gps1_vel_kf_state_mps = 0.0f;
			gps1_velocity_mps = 0.0f;
		}
		zupt_trigger_count++;
		return;
	}

	/* 2. Compute Rotation and transform to Nav Frame */
	const float deg2rad = 0.017453292519943295f;
	float roll_rad = imu1_roll_deg * deg2rad;
	float pitch_rad = imu1_pitch_deg * deg2rad;
	float yaw_rad = 0;

	float cr = cosf(roll_rad);
	float sr = sinf(roll_rad);
	float cp = cosf(pitch_rad);
	float sp = sinf(pitch_rad);
	float cy = cosf(yaw_rad);
	float sy = sinf(yaw_rad);

	/* Rotation Matrix: Body -> Navigation Frame */
	float R00 = cy * cp;
	float R01 = cy * sp * sr - sy * cr;
	float R02 = cy * sp * cr + sy * sr;
	float R10 = sy * cp;
	float R11 = sy * sp * sr + cy * cr;
	float R12 = sy * sp * cr - cy * sr;
	float R20 = -sp;
	float R21 = cp * sr;
	float R22 = cp * cr;

	/* Body accelerations in m/s^2 */
	float ax_b = imu_ax_corr_g * IMU_FD_GRAVITY_MPS2;
	float ay_b = imu_ay_corr_g * IMU_FD_GRAVITY_MPS2;
	float az_b = imu_az_corr_g * IMU_FD_GRAVITY_MPS2;

	/* Map-frame true horizontal and vertical accelerations */
	float ax_n = (R00 * ax_b) + (R01 * ay_b) + (R02 * az_b); // True Earth East
	float ay_n = (R10 * ax_b) + (R11 * ay_b) + (R12 * az_b); // True Earth North
	float az_n = (R20 * ax_b) + (R21 * ay_b) + (R22 * az_b); // True Vertical

	/* Strip clean 1g gravity offset out of the vertical component */
	float az_n_lin = az_n - IMU_FD_GRAVITY_MPS2;

	/* 3. Integrate Directly in the Normalized Navigation Frame */
	if (gps_locked)
	{
		/* Mode 1: Fused with GPS Kalman Filter */
		imu_fd_vel_x_mps = gps1_velocity_mps;
		/* Integrate Earth-Frame lateral/vertical components with leaky damping to mitigate drift */
		// Force every mode to use the isolated, rotated nav components!
		imu_fd_vel_y_mps = (0.995f * imu_fd_vel_y_mps) + (ay_n * dt_s);
		imu_fd_vel_z_mps = (0.995f * imu_fd_vel_z_mps) + (az_n_lin * dt_s);
	}
	else if (sdu_speeds_available)
	{
		/* Mode 2: Fused with SDU Wheel Encoder Speed */
		float K = gps1_vel_kf_P / (gps1_vel_kf_P + 1.0f);

		/* Use flat horizontal map acceleration ax_n for state update */
		gps1_vel_kf_state_mps += ax_n * dt_s;
		gps1_vel_kf_state_mps += K * (average_wheel_speed_mps - gps1_vel_kf_state_mps);
		gps1_vel_kf_P = (1.0f - K) * (gps1_vel_kf_P + 0.1f);

		imu_fd_vel_x_mps = gps1_vel_kf_state_mps;
		// Force every mode to use the isolated, rotated nav components!
		imu_fd_vel_y_mps = (0.995f * imu_fd_vel_y_mps) + (ay_n * dt_s);
		imu_fd_vel_z_mps = (0.995f * imu_fd_vel_z_mps) + (az_n_lin * dt_s);
		gps1_velocity_mps = gps1_vel_kf_state_mps;
	}
	else
	{
		/* Mode 3: Pure Inertial Flat Plane Integration */
		/* Direct integration of isolated gravity-free map-frame accelerations */
		imu_fd_vel_x_mps = (0.995f * imu_fd_vel_x_mps) + (ax_n * dt_s);
		// Force every mode to use the isolated, rotated nav components!
		imu_fd_vel_y_mps = (0.995f * imu_fd_vel_y_mps) + (ay_n * dt_s);
		imu_fd_vel_z_mps = (0.995f * imu_fd_vel_z_mps) + (az_n_lin * dt_s);

		/* Sync tracking states */
		gps1_vel_kf_state_mps = imu_fd_vel_x_mps;
		gps1_velocity_mps = imu_fd_vel_x_mps;

		if (gps1_vel_kf_P < 5.0f)
		{
			gps1_vel_kf_P += 0.05f * dt_s;
		}
	}
}

static void CAN_SendImuFd(uint32_t now_ms)
{
	uint32_t ts_us;
	uint16_t error_flags;
	float gx_now, gy_now, gz_now;
	float ang_a, ang_b, ang_c;
	int16_t ax_mg, ay_mg, az_mg;
	int16_t aa, ab, ac;
	int16_t vx, vy, vz;
	int16_t gx, gy, gz;
	uint32_t actual_dt_us;
	uint32_t expected_dt_us;
	uint16_t jitter;

	ts_us = now_ms * 1000U;

	error_flags = 0U;
	if (can_imu_comm_ok == 0U)
	{
		error_flags |= 0x0001U;
	}
	if (can_imu_init_ok == 0U)
	{
		error_flags |= 0x0002U;
	}
	if (imu_cal_done == 0U)
	{
		error_flags |= 0x0004U;
	}
	error_flags |= (uint16_t)((imu_fd_msg_counter & 0x07U) << 7U);

	/* angular acceleration: finite difference of corrected gyro */
	gx_now = imu_gx_corr_dps;
	gy_now = imu_gy_corr_dps;
	gz_now = imu_gz_corr_dps;
	ang_a = (gx_now - imu_fd_prev_gx_dps) / IMU_FD_DT_S;
	ang_b = (gy_now - imu_fd_prev_gy_dps) / IMU_FD_DT_S;
	ang_c = (gz_now - imu_fd_prev_gz_dps) / IMU_FD_DT_S;
	imu_fd_prev_gx_dps = gx_now;
	imu_fd_prev_gy_dps = gy_now;
	imu_fd_prev_gz_dps = gz_now;

	/* linear velocity: integrate using multi-mode sensor fusion and ZUPT fallbacks */
	CAN_UpdateVelocity(IMU_FD_DT_S, now_ms);

	ax_mg = CAN_ClampS16(imu_ax_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);
	ay_mg = CAN_ClampS16(imu_ay_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);
	az_mg = CAN_ClampS16(imu_az_corr_g * IMU1_ACCEL_CAN_SCALE_MG_PER_G);
	aa = CAN_ClampS16(ang_a * IMU_FD_ANG_ACCEL_SCALE);
	ab = CAN_ClampS16(ang_b * IMU_FD_ANG_ACCEL_SCALE);
	ac = CAN_ClampS16(ang_c * IMU_FD_ANG_ACCEL_SCALE);
	vx = CAN_ClampS16(imu_fd_vel_x_mps * IMU_FD_LIN_VEL_SCALE);
	vy = CAN_ClampS16(imu_fd_vel_y_mps * IMU_FD_LIN_VEL_SCALE);
	vz = CAN_ClampS16(imu_fd_vel_z_mps * IMU_FD_LIN_VEL_SCALE);
	gx = CAN_ClampS16(gx_now * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	gy = CAN_ClampS16(gy_now * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);
	gz = CAN_ClampS16(gz_now * IMU1_ATT_CAN_SCALE_CDEG_PER_DEG);

	if (imu_fd_half == 0U)
	{
		/* store sample 1 and return — will send on next call */
		imu_fd_s1_ts_us = ts_us;
		imu_fd_s1_ax = ax_mg;
		imu_fd_s1_ay = ay_mg;
		imu_fd_s1_az = az_mg;
		imu_fd_s1_aa = aa;
		imu_fd_s1_ab = ab;
		imu_fd_s1_ac = ac;
		imu_fd_s1_vx = vx;
		imu_fd_s1_vy = vy;
		imu_fd_s1_vz = vz;
		imu_fd_s1_gx = gx;
		imu_fd_s1_gy = gy;
		imu_fd_s1_gz = gz;
		imu_fd_half = 1U;
		return;
	}

	/* compute jitter: |actual_dt - expected_dt| */
	actual_dt_us = (ts_us >= imu_fd_s1_ts_us) ? (ts_us - imu_fd_s1_ts_us) : 0U;
	expected_dt_us = (uint32_t)IMU_FD_EXPECTED_PERIOD * 100U;
	jitter = (uint16_t)((actual_dt_us > expected_dt_us) ? ((actual_dt_us - expected_dt_us) & 0xFFFFU) : ((expected_dt_us - actual_dt_us) & 0xFFFFU));

	memset(can_tx_data, 0, sizeof(can_tx_data));

	/* header */
	CAN_PackU32LE(can_tx_data, 0U, imu_fd_s1_ts_us);
	can_tx_data[4] = IMU_FD_EXPECTED_PERIOD;
	CAN_PackU16LE(can_tx_data, 5U, error_flags);

	/* sample 1 */
	CAN_PackS16LE(can_tx_data, 7U, imu_fd_s1_ax);
	CAN_PackS16LE(can_tx_data, 9U, imu_fd_s1_ay);
	CAN_PackS16LE(can_tx_data, 11U, imu_fd_s1_az);
	CAN_PackS16LE(can_tx_data, 13U, imu_fd_s1_aa);
	CAN_PackS16LE(can_tx_data, 15U, imu_fd_s1_ab);
	CAN_PackS16LE(can_tx_data, 17U, imu_fd_s1_ac);
	CAN_PackS16LE(can_tx_data, 19U, imu_fd_s1_vx);
	CAN_PackS16LE(can_tx_data, 21U, imu_fd_s1_vy);
	CAN_PackS16LE(can_tx_data, 23U, imu_fd_s1_vz);
	CAN_PackS16LE(can_tx_data, 25U, imu_fd_s1_gx);
	CAN_PackS16LE(can_tx_data, 27U, imu_fd_s1_gy);
	CAN_PackS16LE(can_tx_data, 29U, imu_fd_s1_gz);

	/* jitter */
	CAN_PackU16LE(can_tx_data, 31U, jitter);

	/* sample 2 */
	CAN_PackS16LE(can_tx_data, 33U, ax_mg);
	CAN_PackS16LE(can_tx_data, 35U, ay_mg);
	CAN_PackS16LE(can_tx_data, 37U, az_mg);
	CAN_PackS16LE(can_tx_data, 39U, aa);
	CAN_PackS16LE(can_tx_data, 41U, ab);
	CAN_PackS16LE(can_tx_data, 43U, ac);
	CAN_PackS16LE(can_tx_data, 45U, vx);
	CAN_PackS16LE(can_tx_data, 47U, vy);
	CAN_PackS16LE(can_tx_data, 49U, vz);
	CAN_PackS16LE(can_tx_data, 51U, gx);
	CAN_PackS16LE(can_tx_data, 53U, gy);
	CAN_PackS16LE(can_tx_data, 55U, gz);

	/* bytes 57-63 unused, already zeroed */

	imu_fd_half = 0U;
	imu_fd_msg_counter = (imu_fd_msg_counter + 1U) & 0x07U;
	(void)CAN_Send(IMU_FD_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *fdcan)
{
	FDCAN_FilterTypeDef sFilterConfig = {0};

	if (fdcan == NULL)
	{
		return HAL_ERROR;
	}

	can_fdcan = fdcan;

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x7FF;

	if (HAL_FDCAN_ConfigFilter(can_fdcan, &sFilterConfig) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (HAL_FDCAN_Start(can_fdcan) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (HAL_FDCAN_ActivateNotification(can_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
									   0) != HAL_OK)
	{
		return HAL_ERROR;
	}

	can_tx_header.IdType = FDCAN_STANDARD_ID;
	can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
	can_tx_header.DataLength = FDCAN_DLC_BYTES_8;
	can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	can_tx_header.MessageMarker = 0U;

	fdcan_tx_count = 0U;
	fdcan_rx_count = 0U;
	fdcan_rx_error_count = 0U;
	fdcan1_debug_cb = 0U;

	can_imu_comm_ok = 0U;
	can_imu_init_ok = 0U;

	last_imu1_accel_tx_time = 0U;
	last_imu1_att_tx_time = 0U;
	last_gps1_fd_tx_time = 0U;
	last_gps_cog_data_tx_time = 0U;

	last_imu_fd_tx_time = 0U;
	imu_fd_half = 0U;
	imu_fd_prev_gx_dps = 0.0f;
	imu_fd_prev_gy_dps = 0.0f;
	imu_fd_prev_gz_dps = 0.0f;
	imu_fd_vel_x_mps = 0.0f;
	imu_fd_vel_y_mps = 0.0f;
	imu_fd_vel_z_mps = 0.0f;

	return HAL_OK;
}

void CAN_SetImuStatus(uint8_t imu_comm_ok, uint8_t imu_init_ok)
{
	can_imu_comm_ok = imu_comm_ok;
	can_imu_init_ok = imu_init_ok;
}

void CAN_Process(uint32_t now_ms)
{
	if ((now_ms - last_imu_fd_tx_time) >= IMU_FD_TX_INTERVAL_MS)
	{
		last_imu_fd_tx_time = now_ms;
		CAN_SendImuFd(now_ms);
	}

	if ((now_ms - last_imu1_accel_tx_time) >= IMU1_ACCEL_TX_INTERVAL_MS)
	{
		last_imu1_accel_tx_time = now_ms;
		CAN_SendImuAccel();
	}

	if ((now_ms - last_imu1_att_tx_time) >= IMU1_ATT_TX_INTERVAL_MS)
	{
		last_imu1_att_tx_time = now_ms;
		CAN_SendImuAtt();
	}

#ifdef SMU_GPS_IMU_CROSS
	if (SMU_BOARD_ID == 0U)
	{
		if ((now_ms - last_gps1_fd_tx_time) >= GPS_COG_TIMESYNC_TX_INTERVAL_MS)
		{
			last_gps1_fd_tx_time = now_ms;
			CAN_SendGpsFd(now_ms);
		}

		if ((now_ms - last_gps_cog_data_tx_time) >= GPS_COG_DATA_TX_INTERVAL_MS)
		{
			last_gps_cog_data_tx_time = now_ms;
			CAN_SendGpsCogPos(now_ms);
			CAN_SendGpsCogNav(now_ms);
			/* Note: CAN_SendGpsCogImu is commented out because IMU FD frame is
			 * already transmitted on the same ID (0x043) in the new format. */
			// CAN_SendGpsCogImu(now_ms);
		}
	}
#endif /* SMU_GPS_IMU_CROSS */
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (hfdcan == can_fdcan && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
	{
		FDCAN_RxHeaderTypeDef RxHeader;
		uint8_t RxData[64];

		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
		{
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			{
				fdcan_rx_count++;

				// Check if this is an SDU wheel speed frame (0x084, 0x08C, 0x094, 0x09C)
				if (RxHeader.Identifier == 0x084 || RxHeader.Identifier == 0x08C ||
					RxHeader.Identifier == 0x094 || RxHeader.Identifier == 0x09C)
				{

					uint8_t idx = (RxHeader.Identifier - 0x084) / 8;
					if (RxHeader.DataLength == FDCAN_DLC_BYTES_64)
					{
						uint16_t raw_rpm = (uint16_t)(RxData[6] | (RxData[7] << 8));
						sdu_wheel_rpm[idx] = (float)raw_rpm / 10.0f;
						last_sdu_rx_time[idx] = HAL_GetTick();
					}
				}
			}
		}
	}
}
