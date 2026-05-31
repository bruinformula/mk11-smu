#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define IMU_WHO_AM_I_VALUE  0x6C

HAL_StatusTypeDef IMU_Init(SPI_HandleTypeDef *spi);
HAL_StatusTypeDef IMU_CheckWhoAmI(uint8_t *whoami);
HAL_StatusTypeDef IMU_ReadAxes(int16_t *gx, int16_t *gy, int16_t *gz,
		int16_t *ax, int16_t *ay, int16_t *az);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
