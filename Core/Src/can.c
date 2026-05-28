#include "can.h"
#include "state.h"
#include "gps.h"

#include <string.h>

#define IMU1_ACCEL_TX_INTERVAL_MS 10U
#define IMU1_ATT_TX_INTERVAL_MS   10U
#define GPS_COG_TIMESYNC_TX_INTERVAL_MS 100U
#define GPS_COG_DATA_TX_INTERVAL_MS      50U
#define CAN_GPS_COG_FRAME_BYTES          64U
#define CAN_GPS_ERROR_NO_FIX             0x01U
#define CAN_GPS_ERROR_UART               0x02U
#define CAN_GPS_ERROR_NO_SENTENCES       0x04U

volatile uint32_t fdcan_tx_count = 0U;
volatile uint32_t fdcan_rx_count = 0U;
volatile uint32_t fdcan_rx_error_count = 0U;
volatile uint32_t fdcan1_debug_cb = 0U;

static FDCAN_HandleTypeDef *can_fdcan = NULL;
static FDCAN_TxHeaderTypeDef can_tx_header;
static uint8_t can_tx_data[CAN_GPS_COG_FRAME_BYTES];

static uint32_t last_imu1_accel_tx_time = 0U;
static uint32_t last_imu1_att_tx_time = 0U;
static uint32_t last_gps_cog_timesync_tx_time = 0U;
static uint32_t last_gps_cog_data_tx_time = 0U;

static uint8_t can_imu_comm_ok = 0U;
static uint8_t can_imu_init_ok = 0U;

static int16_t CAN_ClampS16(float value) {
	if (value > 32767.0f) {
		return 32767;
	}

	if (value < -32768.0f) {
		return -32768;
	}

	return (int16_t) value;
}

static uint16_t CAN_ClampU16(float value) {
	if (value > 65535.0f) {
		return 65535U;
	}

	if (value < 0.0f) {
		return 0U;
	}

	return (uint16_t) value;
}

static uint32_t CAN_ClampU32(float value) {
	if (value > 4294967295.0f) {
		return 4294967295UL;
	}

	if (value < 0.0f) {
		return 0U;
	}

	return (uint32_t) value;
}

static int32_t CAN_ClampS32(float value) {
	if (value > 2147483647.0f) {
		return 2147483647;
	}

	if (value < -2147483648.0f) {
		return (-2147483647 - 1);
	}

	return (int32_t) value;
}

static void CAN_PackS16LE(uint8_t *data, uint8_t idx, int16_t value) {
	data[idx] = (uint8_t) (value & 0xFF);
	data[idx + 1U] = (uint8_t) (((uint16_t) value >> 8) & 0xFFU);
}

static void CAN_PackU16LE(uint8_t *data, uint8_t idx, uint16_t value) {
	data[idx] = (uint8_t) (value & 0xFFU);
	data[idx + 1U] = (uint8_t) ((value >> 8) & 0xFFU);
}

static void CAN_PackS32LE(uint8_t *data, uint8_t idx, int32_t value) {
	data[idx] = (uint8_t) (value & 0xFF);
	data[idx + 1U] = (uint8_t) (((uint32_t) value >> 8) & 0xFFU);
	data[idx + 2U] = (uint8_t) (((uint32_t) value >> 16) & 0xFFU);
	data[idx + 3U] = (uint8_t) (((uint32_t) value >> 24) & 0xFFU);
}

static void CAN_PackU32LE(uint8_t *data, uint8_t idx, uint32_t value) {
	data[idx] = (uint8_t) (value & 0xFFU);
	data[idx + 1U] = (uint8_t) ((value >> 8) & 0xFFU);
	data[idx + 2U] = (uint8_t) ((value >> 16) & 0xFFU);
	data[idx + 3U] = (uint8_t) ((value >> 24) & 0xFFU);
}

static uint8_t CAN_Digit(char value) {
	if ((value >= '0') && (value <= '9')) {
		return (uint8_t) (value - '0');
	}

	return 0U;
}

static uint32_t CAN_ParseUtcMsOfDay(void) {
	char utc_time[16];
	uint32_t hours;
	uint32_t minutes;
	uint32_t seconds;
	uint32_t milliseconds = 0U;

	memcpy(utc_time, (const void *)gps_data.utc_time, sizeof(utc_time));

	if ((utc_time[0] == '\0') || (utc_time[1] == '\0')
			|| (utc_time[2] == '\0') || (utc_time[3] == '\0')
			|| (utc_time[4] == '\0') || (utc_time[5] == '\0')) {
		return 0U;
	}

	hours = ((uint32_t)CAN_Digit(utc_time[0]) * 10U) + CAN_Digit(utc_time[1]);
	minutes = ((uint32_t)CAN_Digit(utc_time[2]) * 10U) + CAN_Digit(utc_time[3]);
	seconds = ((uint32_t)CAN_Digit(utc_time[4]) * 10U) + CAN_Digit(utc_time[5]);

	if (utc_time[6] == '.') {
		milliseconds = ((uint32_t)CAN_Digit(utc_time[7]) * 100U)
				+ ((uint32_t)CAN_Digit(utc_time[8]) * 10U)
				+ CAN_Digit(utc_time[9]);
	}

	return (((hours * 60U) + minutes) * 60U + seconds) * 1000U + milliseconds;
}

static uint32_t CAN_ParseUtcDateYymmdd(void) {
	char utc_date[16];
	uint32_t date = 0U;
	uint8_t idx;

	memcpy(utc_date, (const void *)gps_data.utc_date, sizeof(utc_date));

	for (idx = 0U; idx < 6U; idx++) {
		if ((utc_date[idx] < '0') || (utc_date[idx] > '9')) {
			return 0U;
		}

		date = (date * 10U) + CAN_Digit(utc_date[idx]);
	}

	return date;
}

static uint8_t CAN_GpsErrorFlags(void) {
	uint8_t flags = 0U;

	if (gps_data.fix_valid == 0U) {
		flags |= CAN_GPS_ERROR_NO_FIX;
	}

	if (gps_diag.uart_last_error_code != 0U) {
		flags |= CAN_GPS_ERROR_UART;
	}

	if (gps_diag.sentence_count == 0U) {
		flags |= CAN_GPS_ERROR_NO_SENTENCES;
	}

	return flags;
}

static HAL_StatusTypeDef CAN_Send(uint32_t id, uint32_t data_length,
		uint32_t fd_format) {
	if (can_fdcan == NULL) {
		return HAL_ERROR;
	}

	can_tx_header.Identifier = id;
	can_tx_header.DataLength = data_length;
	can_tx_header.FDFormat = fd_format;

	if (HAL_FDCAN_AddMessageToTxFifoQ(can_fdcan, &can_tx_header, can_tx_data)
			== HAL_OK) {
		fdcan_tx_count++;
		return HAL_OK;
	}

	return HAL_ERROR;
}

static void CAN_SendImuAccel(void) {
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

	(void) CAN_Send(IMU1_ACCEL_TX_ID, FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN);
}

static void CAN_SendImuAtt(void) {
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

	(void) CAN_Send(IMU1_ATT_TX_ID, FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN);
}

/*
 * Provisional 64-byte GPS COG SMU payloads. The lookup CSV defines IDs,
 * rates, and DLC 64, but the message-format CSV does not define GPS-specific
 * byte offsets yet.
 */
static void CAN_SendGpsCogTimesync(uint32_t now_ms) {
	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackU32LE(can_tx_data, 0U, now_ms * 1000U);
	CAN_PackU32LE(can_tx_data, 4U, CAN_ParseUtcMsOfDay());
	CAN_PackU32LE(can_tx_data, 8U, CAN_ParseUtcDateYymmdd());
	can_tx_data[12] = gps_data.fix_valid;
	can_tx_data[13] = gps_data.fix_quality;
	can_tx_data[14] = gps_data.satellites;
	can_tx_data[15] = gps_data.heading_valid;
	CAN_PackU32LE(can_tx_data, 16U, gps_diag.sentence_count);
	CAN_PackU32LE(can_tx_data, 20U, gps_diag.rmc_count);
	CAN_PackU32LE(can_tx_data, 24U, gps_diag.gga_count);
	CAN_PackU32LE(can_tx_data, 28U, gps_diag.pqtmtar_count);
	can_tx_data[63] = CAN_GpsErrorFlags();

	(void) CAN_Send(GPS_COG_TIMESYNC_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

static void CAN_SendGpsCogPos(uint32_t now_ms) {
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

	(void) CAN_Send(GPS_COG_POS_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

static void CAN_SendGpsCogNav(uint32_t now_ms) {
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

	(void) CAN_Send(GPS_COG_NAV_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

static void CAN_SendGpsCogImu(uint32_t now_ms) {
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

	(void) CAN_Send(GPS_COG_IMU_TX_ID, FDCAN_DLC_BYTES_64, FDCAN_FD_CAN);
}

HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *fdcan) {
	FDCAN_FilterTypeDef sFilterConfig = { 0 };

	if (fdcan == NULL) {
		return HAL_ERROR;
	}

	can_fdcan = fdcan;

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x7FF;

	if (HAL_FDCAN_ConfigFilter(can_fdcan, &sFilterConfig) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_FDCAN_Start(can_fdcan) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_FDCAN_ActivateNotification(can_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
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
	last_gps_cog_timesync_tx_time = 0U;
	last_gps_cog_data_tx_time = 0U;

	return HAL_OK;
}

void CAN_SetImuStatus(uint8_t imu_comm_ok, uint8_t imu_init_ok) {
	can_imu_comm_ok = imu_comm_ok;
	can_imu_init_ok = imu_init_ok;
}

void CAN_Process(uint32_t now_ms) {
	if ((now_ms - last_imu1_accel_tx_time) >= IMU1_ACCEL_TX_INTERVAL_MS) {
		last_imu1_accel_tx_time = now_ms;
		CAN_SendImuAccel();
	}

	if ((now_ms - last_imu1_att_tx_time) >= IMU1_ATT_TX_INTERVAL_MS) {
		last_imu1_att_tx_time = now_ms;
		CAN_SendImuAtt();
	}

	if ((now_ms - last_gps_cog_timesync_tx_time) >= GPS_COG_TIMESYNC_TX_INTERVAL_MS) {
		last_gps_cog_timesync_tx_time = now_ms;
		CAN_SendGpsCogTimesync(now_ms);
	}

	if ((now_ms - last_gps_cog_data_tx_time) >= GPS_COG_DATA_TX_INTERVAL_MS) {
		last_gps_cog_data_tx_time = now_ms;
		CAN_SendGpsCogPos(now_ms);
		CAN_SendGpsCogNav(now_ms);
		CAN_SendGpsCogImu(now_ms);
	}
}
