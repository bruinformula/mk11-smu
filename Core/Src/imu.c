#include "imu.h"
#include "main.h"

#define LSM6DSO_WHO_AM_I_REG   0x0F
#define LSM6DSO_CTRL1_XL       0x10
#define LSM6DSO_CTRL2_G        0x11
#define LSM6DSO_CTRL3_C        0x12
#define LSM6DSO_OUTX_L_G       0x22
#define LSM6DSO_OUTX_L_A       0x28

static SPI_HandleTypeDef *imu_spi = NULL;

static void IMU_CS_Low(void) {
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void IMU_CS_High(void) {
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef IMU_ReadReg(uint8_t reg, uint8_t *value) {
	HAL_StatusTypeDef status;
	uint8_t tx[2];
	uint8_t rx[2];

	if ((imu_spi == NULL) || (value == NULL)) {
		return HAL_ERROR;
	}

	tx[0] = 0x80U | reg;
	tx[1] = 0x00U;
	rx[0] = 0U;
	rx[1] = 0U;

	IMU_CS_High();
	HAL_Delay(1);
	IMU_CS_Low();

	status = HAL_SPI_TransmitReceive(imu_spi, tx, rx, 2U, HAL_MAX_DELAY);

	IMU_CS_High();

	if (status == HAL_OK) {
		*value = rx[1];
	}

	return status;
}

static HAL_StatusTypeDef IMU_WriteReg(uint8_t reg, uint8_t value) {
	HAL_StatusTypeDef status;
	uint8_t tx[2];

	if (imu_spi == NULL) {
		return HAL_ERROR;
	}

	tx[0] = reg & 0x7FU;
	tx[1] = value;

	IMU_CS_High();
	HAL_Delay(1);
	IMU_CS_Low();

	status = HAL_SPI_Transmit(imu_spi, tx, 2U, HAL_MAX_DELAY);

	IMU_CS_High();

	return status;
}

static HAL_StatusTypeDef IMU_ReadMulti(uint8_t start_reg, uint8_t *data,
		uint16_t len) {
	HAL_StatusTypeDef status;
	uint8_t reg;

	if ((imu_spi == NULL) || (data == NULL) || (len == 0U)) {
		return HAL_ERROR;
	}

	reg = 0x80U | start_reg;

	IMU_CS_High();
	HAL_Delay(1);
	IMU_CS_Low();

	status = HAL_SPI_Transmit(imu_spi, &reg, 1U, HAL_MAX_DELAY);
	if (status == HAL_OK) {
		status = HAL_SPI_Receive(imu_spi, data, len, HAL_MAX_DELAY);
	}

	IMU_CS_High();

	return status;
}

HAL_StatusTypeDef IMU_CheckWhoAmI(uint8_t *whoami) {
	return IMU_ReadReg(LSM6DSO_WHO_AM_I_REG, whoami);
}

HAL_StatusTypeDef IMU_Init(SPI_HandleTypeDef *spi) {
	HAL_StatusTypeDef status;

	if (spi == NULL) {
		return HAL_ERROR;
	}

	imu_spi = spi;

	status = IMU_WriteReg(LSM6DSO_CTRL3_C, 0x44U);
	if (status != HAL_OK) {
		return status;
	}

	status = IMU_WriteReg(LSM6DSO_CTRL1_XL, 0x40U);
	if (status != HAL_OK) {
		return status;
	}

	status = IMU_WriteReg(LSM6DSO_CTRL2_G, 0x40U);
	if (status != HAL_OK) {
		return status;
	}

	return HAL_OK;
}

HAL_StatusTypeDef IMU_ReadAxes(int16_t *gx, int16_t *gy, int16_t *gz,
		int16_t *ax, int16_t *ay, int16_t *az) {
	HAL_StatusTypeDef status;
	uint8_t raw[12];

	if ((gx == NULL) || (gy == NULL) || (gz == NULL) || (ax == NULL)
			|| (ay == NULL) || (az == NULL)) {
		return HAL_ERROR;
	}

	status = IMU_ReadMulti(LSM6DSO_OUTX_L_G, raw, 12U);
	if (status != HAL_OK) {
		return status;
	}

	*gx = (int16_t) ((uint16_t) raw[1] << 8 | raw[0]);
	*gy = (int16_t) ((uint16_t) raw[3] << 8 | raw[2]);
	*gz = (int16_t) ((uint16_t) raw[5] << 8 | raw[4]);
	*ax = (int16_t) ((uint16_t) raw[7] << 8 | raw[6]);
	*ay = (int16_t) ((uint16_t) raw[9] << 8 | raw[8]);
	*az = (int16_t) ((uint16_t) raw[11] << 8 | raw[10]);

	return HAL_OK;
}
