#ifndef CAN_H
#define CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define GPS1_POS_TX_ID   0x4F3U
#define GPS1_NAV_TX_ID   0x4F4U
#define IMU1_ACCEL_TX_ID 0x4F5U
#define IMU1_ATT_TX_ID   0x4F6U

#define IMU1_ACCEL_CAN_SCALE_MG_PER_G       (1000.0f)
#define IMU1_ATT_CAN_SCALE_CDEG_PER_DEG     (100.0f)
#define GPS1_VEL_CAN_SCALE_CMPS_PER_MPS     (100.0f)
#define GPS1_HEADING_CAN_SCALE_CDEG_PER_DEG (100.0f)
#define GPS1_ALT_CAN_SCALE_DM_PER_M         (10.0f)
#define GPS1_POS_CAN_SCALE_DEGE7_PER_DEG    (10000000.0f)

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
