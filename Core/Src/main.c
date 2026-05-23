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
#include <stdio.h>
#include <string.h>

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
/* #define GPS1_RST_DEBUG_HOLD */
#define ROVER_BAUD_PROGRAM
#define ROVER_FACTORY_BAUD  921600U
//#define ROVER_TARGET_BAUD   460800U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
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
 * gnss_config_ok
 * gnss_enable_pqtmtar
 * gnss_apply_config_request
 * base_reset_pin_state
 * rover_reset_pin_state
 * base_wakeup_pin_state
 * rover_wakeup_pin_state
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
volatile uint8_t gnss_config_ok = 0U;
volatile uint8_t gnss_enable_pqtmtar = 0U;
volatile uint8_t gnss_apply_config_request = 0U;
volatile uint8_t pps_init_ok = 0U;
volatile uint8_t state_init_ok = 0U;
volatile uint8_t can_init_ok = 0U;
volatile uint8_t base_reset_pin_state = 0U;
volatile uint8_t rover_reset_pin_state = 0U;
volatile uint8_t base_wakeup_pin_state = 0U;
volatile uint8_t rover_wakeup_pin_state = 0U;
volatile uint32_t gps_uart_error_count = 0U;
volatile uint32_t gps_uart_abort_count = 0U;
volatile uint32_t gps_uart_irq_count = 0U;
volatile uint8_t gps_mode_pin_state = 0U;

uint32_t last_imu_poll_time = 0U;
uint32_t last_gps_log_time = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if 0
static void GNSS_RefreshPinStates(void) {
  base_reset_pin_state = (uint8_t) HAL_GPIO_ReadPin(BASE_RST_GPIO_Port,
      BASE_RST_Pin);
  rover_reset_pin_state = (uint8_t) HAL_GPIO_ReadPin(ROVER_RST_GPIO_Port,
      ROVER_RST_Pin);
  base_wakeup_pin_state = (uint8_t) HAL_GPIO_ReadPin(BASE_WKUP_GPIO_Port,
      BASE_WKUP_Pin);
  rover_wakeup_pin_state = (uint8_t) HAL_GPIO_ReadPin(ROVER_WKUP_GPIO_Port,
      ROVER_WKUP_Pin);
}

static void GNSS_SetWakePinsHigh(void) {
  HAL_GPIO_WritePin(BASE_WKUP_GPIO_Port, BASE_WKUP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ROVER_WKUP_GPIO_Port, ROVER_WKUP_Pin, GPIO_PIN_SET);
  GNSS_RefreshPinStates();
}

static void GNSS_SetResetPins(GPIO_PinState base_state,
    GPIO_PinState rover_state) {
  HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, base_state);
  HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, rover_state);
  GNSS_RefreshPinStates();
}

static HAL_StatusTypeDef GNSS_SendCommand(UART_HandleTypeDef *uart,
    const char *command) {
  if ((uart == NULL) || (command == NULL)) {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(uart, (uint8_t*) command,
      (uint16_t) strlen(command), GNSS_COMMAND_TIMEOUT_MS);
}

static HAL_StatusTypeDef GNSS_SendBasePqtmtarCommand(void) {
  const char *command;

  command = (gnss_enable_pqtmtar != 0U) ? GNSS_BASE_PQTMTAR_ON_CMD :
      GNSS_BASE_PQTMTAR_OFF_CMD;

  return GNSS_SendCommand(&huart3, command);
}

static HAL_StatusTypeDef GNSS_ProgramModules(void) {
  HAL_StatusTypeDef status = HAL_OK;

  GNSS_SetWakePinsHigh();

  GNSS_SetResetPins(GPIO_PIN_RESET, GPIO_PIN_SET);
  HAL_Delay(GNSS_BOOT_DELAY_MS);

  if (GNSS_SendCommand(&huart1, GNSS_ROVER_MODE_CMD) != HAL_OK) {
    status = HAL_ERROR;
    goto done;
  }
  HAL_Delay(150);

  if (GNSS_SendCommand(&huart1, GNSS_ROVER_GGA_CMD) != HAL_OK) {
    status = HAL_ERROR;
    goto done;
  }
  HAL_Delay(150);

  if (GNSS_SendCommand(&huart1, GNSS_SAVE_PARAMETERS_CMD) != HAL_OK) {
    status = HAL_ERROR;
    goto done;
  }
  HAL_Delay(150);

  HAL_Delay(GNSS_SAVE_DELAY_MS);

  GNSS_SetResetPins(GPIO_PIN_SET, GPIO_PIN_RESET);
  HAL_Delay(GNSS_BOOT_DELAY_MS);

  if (GNSS_SendCommand(&huart3, GNSS_BASE_MODE_CMD) != HAL_OK) {
    status = HAL_ERROR;
    goto done;
  }
  HAL_Delay(150);

  if (GNSS_SendBasePqtmtarCommand() != HAL_OK) {
    status = HAL_ERROR;
    goto done;
  }
  HAL_Delay(150);

  if (GNSS_SendCommand(&huart3, GNSS_SAVE_PARAMETERS_CMD) != HAL_OK) {
    status = HAL_ERROR;
    goto done;
  }

  HAL_Delay(GNSS_SAVE_DELAY_MS);

done:
  GNSS_SetWakePinsHigh();
  GNSS_SetResetPins(GPIO_PIN_SET, GPIO_PIN_SET);

  return status;
}
#endif

static const char* GPS_QualityText(uint8_t quality) {
  switch (quality) {
  case 0U:
    return "INVALID";
  case 1U:
    return "SPS";
  case 2U:
    return "DGPS";
  case 4U:
    return "RTK_FIXED";
  case 5U:
    return "RTK_FLOAT";
  default:
    return "OTHER";
  }
}

#if 0
static void GPS_LogSnapshotUart2(uint32_t now) {
  char log_buffer[320];
  char utc_time[sizeof(gps_data.utc_time)];
  char utc_date[sizeof(gps_data.utc_date)];
  int log_length;
  const char *fix_text;
  const char *heading_text;

  if ((now - last_gps_log_time) < 1000U) {
    return;
  }

  last_gps_log_time = now;

  snprintf(utc_time, sizeof(utc_time), "%s",
      (gps_data.utc_time[0] != '\0') ? (const char*) gps_data.utc_time : "-");
  snprintf(utc_date, sizeof(utc_date), "%s",
      (gps_data.utc_date[0] != '\0') ? (const char*) gps_data.utc_date : "-");

  fix_text = GPS_QualityText(gps_data.fix_quality);
  heading_text = GPS_QualityText(gps_data.heading_quality);

  log_length = snprintf(log_buffer, sizeof(log_buffer),
      "GPS utc=%s date=%s fix_valid=%u fix_q=%u(%s) sats=%u lat=%.7f lon=%.7f alt_m=%.2f spd_kph=%.2f cog_deg=%.2f hdg_valid=%u hdg_q=%u(%s) hdg_deg=%.3f hdg_acc_deg=%.3f pitch_deg=%.3f baseline_m=%.3f pqtmtar=%lu pps=%lu\r\n",
      utc_time, utc_date, gps_data.fix_valid, gps_data.fix_quality, fix_text,
      gps_data.satellites, gps_data.latitude_deg, gps_data.longitude_deg,
      gps_data.altitude_m, gps_data.speed_kph, gps_data.course_deg,
      gps_data.heading_valid, gps_data.heading_quality, heading_text,
      gps_data.heading_deg, gps_data.heading_accuracy_deg,
      gps_data.pitch_deg, gps_data.baseline_length_m,
      (unsigned long) gps_pqtmtar_count,
      (unsigned long) gps1_pps_count);

  if (log_length > 0) {
    (void) HAL_UART_Transmit(&huart2, (uint8_t*) log_buffer,
        (uint16_t) log_length, 100U);
  }
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if ((huart != NULL) && (huart->Instance == USART3)) {
		gps_uart_irq_count++;
	}
	GPS_UART_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if ((huart != NULL) && (huart->Instance == USART3)) {
		gps_uart_error_count++;
		gps_diag.uart_last_error_code = huart->ErrorCode;
		(void) GPS_StartReceiveIT();
	}
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
	if ((huart != NULL) && (huart->Instance == USART3)) {
		gps_uart_abort_count++;
	}
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
#ifdef ROVER_BAUD_PROGRAM
	/* Step 1: All peripherals initialised above.
     Step 2: BASE_RST is LOW (held in reset by GPIO init).
             ROVER_RST is HIGH (rover running at factory 460800).
     Allow rover to settle before sending commands. */
	HAL_Delay(500U);

	/* Step 3: Program rover UART baud and enable NMEA messages */


//	static const char rover_baud_cmd[] = "$PQTMCFGUART,W,460800*13\r\n";

//	    static const char rover_baud_cmd[] = "$PAIR864,0,0,460800*16\r\n";
//	    static const char rover_baud_cmd[] = "$PAIR864,0,0,921600*10\r\n";
	static const char rover_gga_cmd[]  = "$PAIR062,0,1*3F\r\n";
	static const char rover_rmc_cmd[]  = "$PAIR062,4,1*3B\r\n";

	static const char rover_gga_dis_cmd[]  = "$PAIR062,0,0*3E\r\n";
	static const char rover_rmc_dis_cmd[]  = "$PAIR062,4,0*3A\r\n";
	static const char rover_save_cmd[] = "$PQTMSAVEPAR*5A\r\n";
	static const char restore_cmd[] = "$PQTMRESTOREPAR*13\r\n";
	static const char bs1[] = "$PAIR514*3A";
	static const char bs2[] = "$PAIR513*3D";
	static const char bs3[] = "$PAIR865,0,0*31";
//	static const char bs4[] = ""

	uint32_t i;

//	HAL_UART_Transmit(&huart1, (uint8_t *)restore_cmd,
//			(uint16_t)(sizeof(restore_cmd) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(&huart1, (uint8_t *)rover_save_cmd,
//			(uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(&huart1, (uint8_t *)bs1,
//			(uint16_t)(sizeof(bs1) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(&huart1, (uint8_t *)bs2,
//			(uint16_t)(sizeof(bs2) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(&huart3, (uint8_t *)restore_cmd,
//			(uint16_t)(sizeof(restore_cmd) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(&huart3, (uint8_t *)rover_save_cmd,
//			(uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(&huart3, (uint8_t *)bs1,
//			(uint16_t)(sizeof(bs1) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(&huart3, (uint8_t *)bs2,
//			(uint16_t)(sizeof(bs2) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(&huart3, (uint8_t *)bs3,
//			(uint16_t)(sizeof(bs3) - 1U), 100U);
//	HAL_Delay(1000U);

	/* 3a: Send baud-change command at factory baud (460800) */
//			for (i = 0U; i < 5U; i++)
//			{
//				HAL_UART_Transmit(&huart1, (uint8_t *)rover_baud_cmd,
//						(uint16_t)(sizeof(rover_baud_cmd) - 1U), 100U);
//				HAL_Delay(1000U);
//			}
//			huart3.Init.BaudRate = 921600U;
//
//			HAL_UART_Transmit(&huart1, (uint8_t *)rover_save_cmd,
//					(uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
//			HAL_Delay(1000U);
//
//			(void)HAL_UART_Init(&huart1);
//			/* 3a: Send baud-change command at factory baud (460800) */
//			for (i = 0U; i < 5U; i++)
//			{
//				HAL_UART_Transmit(&huart1, (uint8_t *)rover_baud_cmd,
//						(uint16_t)(sizeof(rover_baud_cmd) - 1U), 100U);
//				HAL_Delay(1000U);
//			}
//
//			HAL_UART_Transmit(&huart1, (uint8_t *)rover_save_cmd,
//					(uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
//			HAL_Delay(1000U);



			/* 3b: Switch MCU USART1 to match rover's new baud */
			//    huart1.Init.BaudRate = 460800U;
			//    (void)HAL_UART_Init(&huart1);

			/* 3c: Enable GGA and RMC at target baud */
			//    HAL_UART_Transmit(&huart1, (uint8_t *)rover_gga_cmd,
			//                      (uint16_t)(sizeof(rover_gga_cmd) - 1U), 100U);
			//    HAL_Delay(200U);
			//    HAL_UART_Transmit(&huart1, (uint8_t *)rover_rmc_cmd,
			//                      (uint16_t)(sizeof(rover_rmc_cmd) - 1U), 100U);
			//    HAL_Delay(200U);

			//    HAL_UART_Transmit(&huart1, (uint8_t *)rover_gga_dis_cmd,
			//                          (uint16_t)(sizeof(rover_gga_dis_cmd) - 1U), 100U);
			//        HAL_Delay(200U);
			//        HAL_UART_Transmit(&huart1, (uint8_t *)rover_rmc_dis_cmd,
			//                          (uint16_t)(sizeof(rover_rmc_dis_cmd) - 1U), 100U);
			//        HAL_Delay(200U);
			//
			//    /* 3d: Save all settings (baud + message rates) */
			//    for (i = 0U; i < 5U; i++)
			//    {
			//      HAL_UART_Transmit(&huart1, (uint8_t *)rover_save_cmd,
			//                        (uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
			//      HAL_Delay(1000U);
			//    }
			//  }



			//        while(1){}
			//        HAL_UART_Transmit(&huart3, (uint8_t *)restore_cmd,
			//                                  (uint16_t)(sizeof(restore_cmd) - 1U), 100U);
			//        HAL_Delay(1000U);

			/* Step 4: Pull ROVER reset LOW */
			HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_RESET);
			HAL_Delay(150U);

			/* Step 5: Release BASE and ROVER reset simultaneously */
			HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET);

			/* Wait for both modules to complete boot */
			HAL_Delay(5000U);

#else /* !ROVER_BAUD_PROGRAM */
			/* Rover already at 921600 — just release both resets */
			HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET);
			HAL_Delay(5000U);
#endif /* ROVER_BAUD_PROGRAM */

			gps_mode_pin_state = (uint8_t)HAL_GPIO_ReadPin(BASE_RST_GPIO_Port, BASE_RST_Pin);

			PPS_ForcePinConfig();

			if (IMU_Init(&hspi1) == HAL_OK) {
				imu_init_ok = 1U;
			} else {
				imu_init_ok = 0U;
			}

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

			//	HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET);
			//	gps_mode_pin_state = (uint8_t) HAL_GPIO_ReadPin(ROVER_RST_GPIO_Port,
			//			ROVER_RST_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			while (1) {
				uint32_t now = HAL_GetTick();

				gps_mode_pin_state = (uint8_t) HAL_GPIO_ReadPin(BASE_RST_GPIO_Port,
						BASE_RST_Pin);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IMU_CS_Pin|BASE_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ROVER_RST_Pin */
  GPIO_InitStruct.Pin = ROVER_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROVER_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BASE_RST_Pin */
  GPIO_InitStruct.Pin = BASE_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BASE_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BASE_1PPS_Pin */
  GPIO_InitStruct.Pin = BASE_1PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BASE_1PPS_GPIO_Port, &GPIO_InitStruct);

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
