#include "imu.h"
#include "main.h"

#define LSM6DSO_WHO_AM_I_REG   0x0F
#define LSM6DSO_CTRL1_XL       0x10
#define LSM6DSO_CTRL2_G        0x11
#define LSM6DSO_CTRL3_C        0x12
#define LSM6DSO_OUTX_L_G       0x22
#define LSM6DSO_OUTX_L_A       0x28

#define IMU_DMA_BURST_LEN      13U   /* 1 addr byte + 12 data bytes */

static SPI_HandleTypeDef *imu_spi = NULL;

static uint8_t imu_dma_tx[IMU_DMA_BURST_LEN];
static uint8_t imu_dma_rx[IMU_DMA_BURST_LEN];
static volatile uint8_t imu_dma_busy = 0U;
static volatile uint8_t imu_dma_data_ready = 0U;
static volatile int16_t imu_dma_gx = 0, imu_dma_gy = 0, imu_dma_gz = 0;
static volatile int16_t imu_dma_ax = 0, imu_dma_ay = 0, imu_dma_az = 0;
static volatile uint32_t imu_dma_err_count = 0U;

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

	for (uint8_t i = 0; i < IMU_DMA_BURST_LEN; i++) {
		imu_dma_tx[i] = 0x00U;
	}
	imu_dma_tx[0] = 0x80U | LSM6DSO_OUTX_L_G;
	imu_dma_busy = 0U;
	imu_dma_data_ready = 0U;

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

uint8_t IMU_IsBusy(void) {
	return imu_dma_busy;
}

HAL_StatusTypeDef IMU_StartReadAxes(void) {
	HAL_StatusTypeDef status;

	if (imu_spi == NULL) {
		return HAL_ERROR;
	}
	if (imu_dma_busy != 0U) {
		return HAL_BUSY;
	}

	imu_dma_tx[0] = 0x80U | LSM6DSO_OUTX_L_G;
	imu_dma_busy = 1U;

	IMU_CS_Low();
	status = HAL_SPI_TransmitReceive_DMA(imu_spi, imu_dma_tx, imu_dma_rx,
			IMU_DMA_BURST_LEN);
	if (status != HAL_OK) {
		IMU_CS_High();
		imu_dma_busy = 0U;
	}
	return status;
}

uint8_t IMU_PollReadAxes(int16_t *gx, int16_t *gy, int16_t *gz,
		int16_t *ax, int16_t *ay, int16_t *az) {
	if ((gx == NULL) || (gy == NULL) || (gz == NULL) || (ax == NULL)
			|| (ay == NULL) || (az == NULL)) {
		return 0U;
	}
	if (imu_dma_data_ready == 0U) {
		return 0U;
	}

	*gx = imu_dma_gx;
	*gy = imu_dma_gy;
	*gz = imu_dma_gz;
	*ax = imu_dma_ax;
	*ay = imu_dma_ay;
	*az = imu_dma_az;

	imu_dma_data_ready = 0U;
	return 1U;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi != imu_spi) {
		return;
	}

	IMU_CS_High();

	/* rx[0] is clocked in during the address phase — discard. */
	uint8_t *raw = &imu_dma_rx[1];
	imu_dma_gx = (int16_t) ((uint16_t) raw[1]  << 8 | raw[0]);
	imu_dma_gy = (int16_t) ((uint16_t) raw[3]  << 8 | raw[2]);
	imu_dma_gz = (int16_t) ((uint16_t) raw[5]  << 8 | raw[4]);
	imu_dma_ax = (int16_t) ((uint16_t) raw[7]  << 8 | raw[6]);
	imu_dma_ay = (int16_t) ((uint16_t) raw[9]  << 8 | raw[8]);
	imu_dma_az = (int16_t) ((uint16_t) raw[11] << 8 | raw[10]);

	imu_dma_data_ready = 1U;
	imu_dma_busy = 0U;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi != imu_spi) {
		return;
	}
	IMU_CS_High();
	imu_dma_err_count++;
	imu_dma_busy = 0U;
}
