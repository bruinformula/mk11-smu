#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* GPS COG SMU IDs from the CAN ID lookup CSV, interpreted MSB-first. */
#define GPS_COG_TIMESYNC_TX_ID 0x040U
#define GPS_COG_POS_TX_ID      0x041U
#define GPS_COG_NAV_TX_ID      0x042U
#define GPS_COG_IMU_TX_ID      0x043U
#define IMU1_ACCEL_TX_ID 0x4F5U
#define IMU1_ATT_TX_ID   0x4F6U
#define IMU_FD_TX_ID     0x12CU

#define IMU1_ACCEL_CAN_SCALE_MG_PER_G       (1000.0f)
#define IMU1_ATT_CAN_SCALE_CDEG_PER_DEG     (100.0f)
#define GPS1_VEL_CAN_SCALE_CMPS_PER_MPS     (100.0f)
#define GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG (100.0f)
#define GPS1_ALT_CAN_SCALE_MM_PER_M         (1000.0f)
#define GPS1_POS_CAN_SCALE_DEGE7_PER_DEG    (10000000.0f)
#define GPS1_HDOP_CAN_SCALE_CENTI_PER_UNIT  (100.0f)
#define GPS1_BASELINE_CAN_SCALE_MM_PER_M    (1000.0f)

/* IMU FD (0x12C) scale factors */
#define IMU_FD_ANG_ACCEL_SCALE  (10.0f)    /* LSB = 0.1 dps/s */
#define IMU_FD_LIN_VEL_SCALE    (100.0f)   /* LSB = 0.01 m/s (cm/s) */

extern volatile uint32_t fdcan_tx_count;
extern volatile uint32_t fdcan_rx_count;
extern volatile uint32_t fdcan_rx_error_count;
extern volatile uint32_t fdcan1_debug_cb;

HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *fdcan);
void CAN_SetImuStatus(uint8_t imu_comm_ok, uint8_t imu_init_ok);
void CAN_Process(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */
