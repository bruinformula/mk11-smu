#include "can.h"
#include "state.h"
#include "gps.h"

#include <string.h>

#define IMU1_ACCEL_TX_INTERVAL_MS 10U
#define IMU1_ATT_TX_INTERVAL_MS   10U
#define GPS1_TX_INTERVAL_MS       100U

volatile uint32_t fdcan_tx_count = 0U;
volatile uint32_t fdcan_rx_count = 0U;
volatile uint32_t fdcan_rx_error_count = 0U;
volatile uint32_t fdcan1_debug_cb = 0U;

static FDCAN_HandleTypeDef *can_fdcan = NULL;
static FDCAN_TxHeaderTypeDef can_tx_header;
static uint8_t can_tx_data[8];

static uint32_t last_imu1_accel_tx_time = 0U;
static uint32_t last_imu1_att_tx_time = 0U;
static uint32_t last_gps1_tx_time = 0U;

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

static HAL_StatusTypeDef CAN_Send(uint32_t id) {
	if (can_fdcan == NULL) {
		return HAL_ERROR;
	}

	can_tx_header.Identifier = id;

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

	(void) CAN_Send(IMU1_ACCEL_TX_ID);
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

	(void) CAN_Send(IMU1_ATT_TX_ID);
}

static void CAN_SendGpsPos(void) {
	int32_t lat_dege7;
	int32_t lon_dege7;

	lat_dege7 = CAN_ClampS32(
			gps1_latitude_deg * GPS1_POS_CAN_SCALE_DEGE7_PER_DEG);
	lon_dege7 = CAN_ClampS32(
			gps1_longitude_deg * GPS1_POS_CAN_SCALE_DEGE7_PER_DEG);

	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackS32LE(can_tx_data, 0U, lat_dege7);
	CAN_PackS32LE(can_tx_data, 4U, lon_dege7);

	(void) CAN_Send(GPS1_POS_TX_ID);
}

static void CAN_SendGpsNav(void) {
	uint16_t vel_cmps;
	int16_t heading_cdeg;
	int16_t alt_dm;

	vel_cmps = CAN_ClampU16(
			gps1_velocity_mps * GPS1_VEL_CAN_SCALE_CMPS_PER_MPS);
	heading_cdeg = CAN_ClampS16(
			gps1_heading_deg * GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG);
	alt_dm = CAN_ClampS16(gps1_altitude_m * GPS1_ALT_CAN_SCALE_DM_PER_M);

	memset(can_tx_data, 0, sizeof(can_tx_data));
	CAN_PackU16LE(can_tx_data, 0U, vel_cmps);
	CAN_PackS16LE(can_tx_data, 2U, heading_cdeg);
	CAN_PackS16LE(can_tx_data, 4U, alt_dm);
	can_tx_data[6] = gps_data.fix_valid;
	can_tx_data[7] = gps_data.satellites;

	(void) CAN_Send(GPS1_NAV_TX_ID);
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
	last_gps1_tx_time = 0U;

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

	if ((now_ms - last_gps1_tx_time) >= GPS1_TX_INTERVAL_MS) {
		last_gps1_tx_time = now_ms;
		CAN_SendGpsPos();
		CAN_SendGpsNav();
	}
}
