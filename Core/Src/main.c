/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"
#include "gps.h"
#include "pps.h"
#include "state.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* USEFUL LIVE EXPRESSIONS TO PROBE:
 *
 * fdcan_tx_count
 *
 * imu_comm_ok
 * imu_init_ok
 * imu_ax_corr_g
 * imu_ay_corr_g
 * imu_az_corr_g
 * imu1_pitch_deg
 * imu1_roll_deg
 *
 * gps_data.fix_valid
 * gps_data.satellites
 * gps1_latitude_deg
 * gps1_longitude_deg
 * gps1_altitude_m
 * gps1_velocity_mps
 * gps1_heading_deg
 *
 * gps1_pps_count
 * gps1_last_pps_ms
 */

volatile uint8_t imu_whoami = 0U;
volatile uint8_t imu_comm_ok = 0U;
volatile uint8_t imu_init_ok = 0U;
volatile uint8_t imu_last_status = 0U;
volatile uint32_t imu_poll_count = 0U;

volatile int16_t imu_gx_raw = 0;
volatile int16_t imu_gy_raw = 0;
volatile int16_t imu_gz_raw = 0;
volatile int16_t imu_ax_raw = 0;
volatile int16_t imu_ay_raw = 0;
volatile int16_t imu_az_raw = 0;

volatile uint8_t gps_init_ok = 0U;
volatile uint8_t gps_rx_start_ok = 0U;
volatile uint8_t pps_init_ok = 0U;
volatile uint8_t state_init_ok = 0U;
volatile uint8_t can_init_ok = 0U;
volatile uint32_t gps_uart_error_count = 0U;
volatile uint32_t gps_uart_abort_count = 0U;
volatile uint32_t gps_uart_irq_count = 0U;

/* GPS2 command-injection debug */
volatile uint8_t gps2_cmd_attempted = 0U;
volatile uint8_t gps2_cmd_tx_ok = 0U;
volatile uint8_t gps2_cmd_reinit_ok = 0U;
volatile uint8_t gps2_cmd_rx_restart_ok = 0U;
volatile uint32_t gps2_cmd_tx_status = 0U;
volatile uint32_t gps2_cmd_reinit_status = 0U;
volatile uint32_t gps2_cmd_rx_restart_status = 0U;
volatile uint32_t gps2_cmd_count = 0U;

/* GPS2 command-effect diagnostic */
volatile uint32_t gps2_diag_before_disable_pqtmtar_count = 0U;
volatile uint32_t gps2_diag_after_disable_pqtmtar_count = 0U;
volatile uint32_t gps2_diag_before_enable_pqtmtar_count = 0U;
volatile uint32_t gps2_diag_after_enable_pqtmtar_count = 0U;
volatile uint32_t gps2_diag_disable_delta = 0U;
volatile uint32_t gps2_diag_enable_delta = 0U;

/* PB10 GPIO diagnostic */
volatile uint8_t gps2_pb10_readback = 0U;
volatile uint32_t gps2_pb10_toggle_count = 0U;

uint32_t last_imu_poll_time = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void GPS2_USART3_TX_Pin_AF(void);
static void GPS2_USART3_TX_Pin_Analog(void);
static void GPS2_PB10_SlowToggleTest(void);
static void GPS_DelayProcess(uint32_t delay_ms);
static HAL_StatusTypeDef GPS2_CommandWindowBegin(void);
static HAL_StatusTypeDef GPS2_CommandWindowEnd(void);
static HAL_StatusTypeDef GPS2_SendCommandWindowed(const char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	gps_uart_irq_count++;
	GPS_UART_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		gps_uart_error_count++;
		(void) GPS_StartReceiveIT();
	}
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		gps_uart_abort_count++;
	}
}

static void GPS2_USART3_TX_Pin_AF(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void GPS2_USART3_TX_Pin_Analog(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void GPS2_PB10_SlowToggleTest(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*
	 * Test PB10 as plain GPIO, not USART.
	 *
	 * gps2_pb10_readback reads the STM32's PB10 input data register.
	 * Expected Live Expression:
	 *   gps2_pb10_readback toggles 1, 0, 1, 0...
	 *   gps2_pb10_toggle_count keeps increasing
	 *
	 * Limitation:
	 * This proves the STM32 PB10 pad/register is toggling.
	 * It does not prove GPS2.RXD1 / pin 21 sees the same signal.
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	while (1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_Delay(50);
		gps2_pb10_readback = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) ? 1U : 0U;
		gps2_pb10_toggle_count++;
		HAL_Delay(450);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_Delay(50);
		gps2_pb10_readback = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) ? 1U : 0U;
		gps2_pb10_toggle_count++;
		HAL_Delay(450);
	}
}

static void GPS_DelayProcess(uint32_t delay_ms)
{
	uint32_t start = HAL_GetTick();

	while ((HAL_GetTick() - start) < delay_ms) {
		GPS_Process();
		HAL_Delay(5);
	}
}

static HAL_StatusTypeDef GPS2_CommandWindowBegin(void)
{
	HAL_StatusTypeDef status;

	/*
	 * Enter a temporary TX/RX window.
	 *
	 * Important difference from the old function:
	 * RX is restarted BEFORE commands are sent and remains active while GPS2
	 * replies. This prevents us from missing immediate ACK/ERROR responses.
	 */

	(void)HAL_UART_AbortReceive(&huart3);
	(void)HAL_UART_DeInit(&huart3);

	huart3.Init.Mode = UART_MODE_TX_RX;
	status = HAL_UART_Init(&huart3);

	gps2_cmd_reinit_status = (uint32_t)status;
	gps2_cmd_reinit_ok = (status == HAL_OK) ? 1U : 0U;

	if (status != HAL_OK) {
		GPS2_USART3_TX_Pin_Analog();
		gps2_cmd_rx_restart_status = (uint32_t)HAL_ERROR;
		gps2_cmd_rx_restart_ok = 0U;
		gps_rx_start_ok = 0U;
		return HAL_ERROR;
	}

	/*
	 * HAL_UART_Init() may leave PB10 analog/high-Z because of our MSP user code.
	 * Enable PB10 AF for the whole command burst.
	 */
	GPS2_USART3_TX_Pin_AF();

	status = GPS_StartReceiveIT();

	gps2_cmd_rx_restart_status = (uint32_t)status;
	gps2_cmd_rx_restart_ok = (status == HAL_OK) ? 1U : 0U;
	gps_rx_start_ok = (status == HAL_OK) ? 1U : 0U;

	return status;
}

static HAL_StatusTypeDef GPS2_SendCommandWindowed(const char *cmd)
{
	HAL_StatusTypeDef tx_status;

	if (cmd == NULL) {
		return HAL_ERROR;
	}

	gps2_cmd_attempted = 1U;
	gps2_cmd_count++;

	tx_status = HAL_UART_Transmit(&huart3,
	                              (uint8_t *)cmd,
	                              (uint16_t)strlen(cmd),
	                              100U);

	gps2_cmd_tx_status = (uint32_t)tx_status;
	gps2_cmd_tx_ok = (tx_status == HAL_OK) ? 1U : 0U;

	return tx_status;
}

static HAL_StatusTypeDef GPS2_CommandWindowEnd(void)
{
	HAL_StatusTypeDef status;

	/*
	 * Leave the UART command window.
	 * Return PB10 to analog/high-Z so MCU TX does not fight GPS1 TXD2 later.
	 */

	GPS2_USART3_TX_Pin_Analog();

	(void)HAL_UART_AbortReceive(&huart3);
	(void)HAL_UART_DeInit(&huart3);

	huart3.Init.Mode = UART_MODE_RX;
	status = HAL_UART_Init(&huart3);

	gps2_cmd_reinit_status = (uint32_t)status;
	gps2_cmd_reinit_ok = (status == HAL_OK) ? 1U : 0U;

	if (status == HAL_OK) {
		status = GPS_StartReceiveIT();
	}

	gps2_cmd_rx_restart_status = (uint32_t)status;
	gps2_cmd_rx_restart_ok = (status == HAL_OK) ? 1U : 0U;
	gps_rx_start_ok = (status == HAL_OK) ? 1U : 0U;

	return status;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ICACHE_Init();
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(50);

  /*
   * PB10 hardware proof test passed:
   * STM32 PB10 physically toggles GPS2.RXD1 / pin 21.
   * Do not call GPS2_PB10_SlowToggleTest() during normal GPS testing.
   */

  /*
   * Firmware-only Rev 2 GPS startup:
   *
   * Actual reworked hardware:
   *   GPS1 TXD2 -> GPS2 RXD1
   *   GPS2 TXD1 -> MCU USART3_RX / PB11
   *
   * During GPS2 command injection, GPS1 is held in reset so GPS1 TXD2 does not
   * fight MCU PB10 on GPS2 RXD1. After GPS2 is configured, PB10 is returned to
   * analog/high-Z and GPS1 is released so the moving-base correction path can run.
   */

  /* Reset both modules first. RESET_N is active-low. */
  HAL_GPIO_WritePin(GPS1_RST_GPIO_Port, GPS1_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPS2_RST_GPIO_Port, GPS2_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(150);

  /* Keep GPS1/Base in reset. Release only GPS2/Rover. */
  HAL_GPIO_WritePin(GPS1_RST_GPIO_Port, GPS1_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPS2_RST_GPIO_Port, GPS2_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);

  /* Wake GPS2/Rover. */
  HAL_GPIO_WritePin(GPS2_WKUP_GPIO_Port, GPS2_WKUP_Pin, GPIO_PIN_SET);
  HAL_Delay(15);
  HAL_GPIO_WritePin(GPS2_WKUP_GPIO_Port, GPS2_WKUP_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);

  PPS_ForcePinConfig();

  /* Start GPS receive BEFORE sending commands so we can catch ACKs. */
  if (GPS_Init(&huart3) == HAL_OK) {
  	gps_init_ok = 1U;
  } else {
  	gps_init_ok = 0U;
  }

  if (GPS_StartReceiveIT() == HAL_OK) {
  	gps_rx_start_ok = 1U;
  } else {
  	gps_rx_start_ok = 0U;
  }

  /* Give receive ISR a moment to run before command injection. */
  HAL_Delay(100);

  /* Configure GPS2 while RX stays alive, so we do not miss immediate ACK/ERROR replies. */
  if (GPS2_CommandWindowBegin() == HAL_OK) {

  	/* Configure GPS2/Rover UART1 to allow both NMEA and RTCM3 on input/output.
  	 * NMEA = bit 0 = 1
  	 * RTCM3 = bit 2 = 4
  	 * 1 | 4 = 5
  	 */
  	(void)GPS2_SendCommandWindowed("$PQTMCFGPROT,W,1,1,5,5*38\r\n");
  	GPS_DelayProcess(1000);

  	/*
  	 * Enable normal GNSS status messages on GPS2/Rover UART1.
  	 * For standard NMEA messages like GGA/RMC/VTG, do NOT include MsgVer.
  	 */
  	(void)GPS2_SendCommandWindowed("$PQTMCFGMSGRATE,W,GGA,1*0A\r\n");
  	GPS_DelayProcess(1000);

  	(void)GPS2_SendCommandWindowed("$PQTMCFGMSGRATE,W,RMC,1*17\r\n");
  	GPS_DelayProcess(1000);

  	(void)GPS2_SendCommandWindowed("$PQTMCFGMSGRATE,W,VTG,1*0E\r\n");
  	GPS_DelayProcess(1000);

  	/*
  	 * Diagnostic test:
  	 * First disable PQTMTAR. If PB10 command injection actually reaches GPS2,
  	 * gps_pqtmtar_count should mostly stop increasing during this window.
  	 */
  	gps2_diag_before_disable_pqtmtar_count = gps_pqtmtar_count;
  	(void)GPS2_SendCommandWindowed("$PQTMCFGMSGRATE,W,PQTMTAR,1,0*08\r\n");
  	GPS_DelayProcess(3000);
  	gps2_diag_after_disable_pqtmtar_count = gps_pqtmtar_count;
  	gps2_diag_disable_delta =
  		gps2_diag_after_disable_pqtmtar_count - gps2_diag_before_disable_pqtmtar_count;

  	/*
  	 * Now re-enable PQTMTAR. If PB10 command injection works, gps_pqtmtar_count
  	 * should start increasing again during this window.
  	 */
  	gps2_diag_before_enable_pqtmtar_count = gps_pqtmtar_count;
  	(void)GPS2_SendCommandWindowed("$PQTMCFGMSGRATE,W,PQTMTAR,1,1*09\r\n");
  	GPS_DelayProcess(3000);
  	gps2_diag_after_enable_pqtmtar_count = gps_pqtmtar_count;
  	gps2_diag_enable_delta =
  		gps2_diag_after_enable_pqtmtar_count - gps2_diag_before_enable_pqtmtar_count;

  	/* Save parameters after restoring PQTMTAR enabled. */
  	(void)GPS2_SendCommandWindowed("$PQTMSAVEPAR*5A\r\n");
  	GPS_DelayProcess(1000);

  	(void)GPS2_CommandWindowEnd();

  } else {
  	gps2_cmd_tx_ok = 0U;
  	gps2_cmd_rx_restart_ok = 0U;
  }

  /*
   * GPS2 UART is confirmed usable at 921600.
   * Now release GPS1/Base so the GPS1 TXD2 -> GPS2 RXD1 correction path
   * can run during normal moving-base operation.
   *
   * PB10 remains analog/high-Z except during GPS2_SendCommandOnce(), so the MCU
   * will not fight GPS1 TXD2 on GPS2 RXD1.
   */
  HAL_GPIO_WritePin(GPS1_RST_GPIO_Port, GPS1_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(500);

  HAL_GPIO_WritePin(GPS1_WKUP_GPIO_Port, GPS1_WKUP_Pin, GPIO_PIN_SET);
  HAL_Delay(15);
  HAL_GPIO_WritePin(GPS1_WKUP_GPIO_Port, GPS1_WKUP_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);

  PPS_ForcePinConfig();

	if (IMU_Init(&hspi1) == HAL_OK) {
		imu_init_ok = 1U;
	} else {
		imu_init_ok = 0U;
	}

	if (PPS_Init() == HAL_OK) {
		pps_init_ok = 1U;
	} else {
		pps_init_ok = 0U;
	}

	if (State_Init() == HAL_OK) {
		state_init_ok = 1U;
	} else {
		state_init_ok = 0U;
	}

	if (CAN_Init(&hfdcan1) == HAL_OK) {
		can_init_ok = 1U;
	} else {
		can_init_ok = 0U;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		uint32_t now = HAL_GetTick();

		PPS_Process(now); // PPS needs to be polled immediately after getting 'now' before GPS handling, otherwise PPS read does not update
		GPS_Process();

		if ((now - last_imu_poll_time) >= 10U) {
			last_imu_poll_time = now;

			imu_last_status = (uint8_t) IMU_CheckWhoAmI((uint8_t*) &imu_whoami);
			imu_comm_ok =
					((imu_last_status == HAL_OK)
							&& (imu_whoami == IMU_WHO_AM_I_VALUE)) ? 1U : 0U;

			if (imu_comm_ok && imu_init_ok) {
				if (IMU_ReadAxes((int16_t*) &imu_gx_raw, (int16_t*) &imu_gy_raw,
						(int16_t*) &imu_gz_raw, (int16_t*) &imu_ax_raw,
						(int16_t*) &imu_ay_raw, (int16_t*) &imu_az_raw)
						!= HAL_OK) {
					imu_last_status = 0xFFU;
				} else {
					State_UpdateFromImuRaw(imu_gx_raw, imu_gy_raw, imu_gz_raw,
							imu_ax_raw, imu_ay_raw, imu_az_raw, now);
				}
			}

			imu_poll_count++;
		}

		CAN_SetImuStatus(imu_comm_ok, imu_init_ok);
		CAN_Process(now);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 11;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 17;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 11;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 17;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin|GPS2_WKUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS1_RST_GPIO_Port, GPS1_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS2_RST_GPIO_Port, GPS2_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS1_WKUP_GPIO_Port, GPS1_WKUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU_CS_Pin GPS2_RST_Pin GPS2_WKUP_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin|GPS2_RST_Pin|GPS2_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS1_ANT_ON_Pin GPS2_ANT_ON_Pin */
  GPIO_InitStruct.Pin = GPS1_ANT_ON_Pin|GPS2_ANT_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS1_RST_Pin GPS1_WKUP_Pin */
  GPIO_InitStruct.Pin = GPS1_RST_Pin|GPS1_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS1_1PPS_Pin */
  GPIO_InitStruct.Pin = GPS1_1PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPS1_1PPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS2_1PPS_Pin */
  GPIO_InitStruct.Pin = GPS2_1PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPS2_1PPS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
