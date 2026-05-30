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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define GPS_HARD_RESET_AND_RESTOREPAR
//#define GPS_HARD_RESET_AND_PMTK104
/* #define GPS_HARD_RESET_AND_HALT */
/* One-shot: write USART3=base(mode 1), USART1=rover(mode 2), persist, soft-reset.
 * Enable for a single flash, observe BASE>$PQTMCFGRCVRMODE,OK in the log,
 * then comment out for normal operation. */
//#define GPS_PROGRAM_RCVR_MODES
//#define ROVER_BAUD_PROGRAM
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
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

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
volatile uint8_t gps_mode_pin_state = 0U;
volatile uint32_t gps_uart_error_count = 0U;
volatile uint32_t gps_uart_abort_count = 0U;
volatile uint32_t gps_uart_irq_count = 0U;

uint32_t last_imu_poll_time = 0U;

/* Rover (USART1) DMA circular RX buffer — sized to hold ~10ms worth of bytes
 * at 921600 baud (920 bytes) with margin. Drained every main-loop iteration. */
#define ROVER_DMA_RX_BUF_SIZE      1024U
#define ROVER_RTCM3_FRAME_BUF_SIZE  512U  /* largest RTCM3 frame we accumulate */
static uint8_t rover_dma_rx_buf[ROVER_DMA_RX_BUF_SIZE];
static uint16_t rover_dma_rx_last_pos = 0U;

typedef enum {
    RTCM3_SEEK = 0, /* hunting for 0xD3 sync                         */
    RTCM3_HDR1,     /* got sync, waiting for length-high byte         */
    RTCM3_HDR2,     /* got length-high, waiting for length-low byte   */
    RTCM3_PAYLOAD   /* accumulating payload + 3 CRC bytes             */
} Rtcm3State_t;

static struct {
    Rtcm3State_t state;
    uint8_t  buf[ROVER_RTCM3_FRAME_BUF_SIZE];
    uint16_t pos;      /* write index into buf[]                      */
    uint16_t expected; /* total bytes in complete frame (payload+6)   */
} rtcm3_parser;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Rover_FeedByte(uint8_t b);
static void Rover_DrainDmaRx(void);
static void Rover_DrainingDelay(uint32_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  GPS_UART_RxHalfCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	GPS_UART_RxCpltCallback(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  GPS_UART_TxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if ((huart != NULL) && (huart->Instance == USART3)) {
		gps_uart_error_count++;
		gps_diag.uart_last_error_code = huart->ErrorCode;
		(void) GPS_StartReceiveIT();
	} else if ((huart != NULL) && (huart->Instance == USART1)) {
		/* Rover USART1 overrun or framing error — clear and restart circular DMA RX.
		 * Without this, HAL stops the DMA on any error and PB7 goes permanently deaf. */
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF
				| UART_CLEAR_NEF | UART_CLEAR_PEF);
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		rover_dma_rx_last_pos = 0U;
		(void) HAL_UART_Receive_DMA(huart, rover_dma_rx_buf, ROVER_DMA_RX_BUF_SIZE);
	}
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
	if ((huart != NULL) && (huart->Instance == USART3)) {
		gps_uart_abort_count++;
	}
}

/* ── RTCM3 frame detector for rover (USART1) stream ─────────────────────────
 * The rover GPS outputs RTCM3 binary on UART1 TX.  Each frame is:
 *   [0xD3][len_hi (2 bits)][len_lo][...payload (len bytes)...][crc0][crc1][crc2]
 * Total = len + 6 bytes.  The three CRC bytes are CRC-24Q over bytes 0..len+2.
 *
 * We detect complete frames and emit a clean ASCII summary line per frame
 * ("R3> 1074 127B") rather than dumping raw binary to USART2.  This avoids
 * the 0x0A/0x0D stripping corruption that was fragmenting frames in the log. */

static uint32_t Rover_Crc24q(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0UL;
    for (uint16_t i = 0U; i < len; i++) {
        crc ^= ((uint32_t)data[i]) << 16;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc <<= 1;
            if ((crc & 0x1000000UL) != 0UL) { crc ^= 0x1864CFBUL; }
        }
    }
    return crc & 0xFFFFFFUL;
}

static void Rover_OnFrame(const uint8_t *frame, uint16_t total_len)
{
    char line[48];
    int  n;
    uint16_t payload_len = total_len - 6U;
    uint16_t msg_type    = (uint16_t)(((uint16_t)frame[3] << 4) | ((uint16_t)frame[4] >> 4));
    uint32_t want_crc    = Rover_Crc24q(frame, total_len - 3U);
    uint32_t got_crc     = ((uint32_t)frame[total_len - 3U] << 16)
                         | ((uint32_t)frame[total_len - 2U] <<  8)
                         |  (uint32_t)frame[total_len - 1U];

    if (want_crc == got_crc) {
        n = snprintf(line, sizeof(line), "R3> %u %uB\r\n", msg_type, payload_len);
    } else {
        n = snprintf(line, sizeof(line), "R3> CRC_ERR %uB\r\n", total_len);
    }
    if (n > 0) {
        GPS_DebugMirror((const uint8_t *)line, (uint16_t)n);
    }
}

static void Rover_FeedByte(uint8_t b)
{
    switch (rtcm3_parser.state) {
    case RTCM3_SEEK:
        if (b == 0xD3U) {
            rtcm3_parser.pos = 0U;
            rtcm3_parser.buf[rtcm3_parser.pos++] = b;
            rtcm3_parser.state = RTCM3_HDR1;
        }
        break;

    case RTCM3_HDR1:
        /* Upper 6 bits of this byte must be zero per RTCM3 spec */
        if ((b & 0xFCU) != 0x00U) {
            rtcm3_parser.state = RTCM3_SEEK;
            if (b == 0xD3U) {           /* could itself be a new sync byte */
                rtcm3_parser.pos = 0U;
                rtcm3_parser.buf[rtcm3_parser.pos++] = b;
                rtcm3_parser.state = RTCM3_HDR1;
            }
            break;
        }
        rtcm3_parser.buf[rtcm3_parser.pos++] = b;
        rtcm3_parser.state = RTCM3_HDR2;
        break;

    case RTCM3_HDR2:
        rtcm3_parser.buf[rtcm3_parser.pos++] = b;
        {
            uint16_t plen = (uint16_t)(((uint16_t)(rtcm3_parser.buf[1] & 0x03U) << 8) | b);
            rtcm3_parser.expected = plen + 6U; /* 3 header + payload + 3 CRC */
            if (rtcm3_parser.expected > ROVER_RTCM3_FRAME_BUF_SIZE) {
                rtcm3_parser.state = RTCM3_SEEK; /* implausibly large, reject */
                break;
            }
        }
        rtcm3_parser.state = RTCM3_PAYLOAD;
        break;

    case RTCM3_PAYLOAD:
        if (rtcm3_parser.pos < ROVER_RTCM3_FRAME_BUF_SIZE) {
            rtcm3_parser.buf[rtcm3_parser.pos++] = b;
        }
        if (rtcm3_parser.pos >= rtcm3_parser.expected) {
            Rover_OnFrame(rtcm3_parser.buf, rtcm3_parser.expected);
            rtcm3_parser.state = RTCM3_SEEK;
        }
        break;

    default:
        rtcm3_parser.state = RTCM3_SEEK;
        break;
    }
}

/* Drain whatever bytes have accumulated in the rover (USART1) circular DMA
 * RX buffer through the RTCM3 frame detector. Safe to call from anywhere
 * (boot init, polling-delay loops, main loop). */
static void Rover_DrainDmaRx(void)
{
    uint16_t dma_pos = (uint16_t)(ROVER_DMA_RX_BUF_SIZE
            - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    if (dma_pos >= ROVER_DMA_RX_BUF_SIZE) { dma_pos = 0U; }
    while (rover_dma_rx_last_pos != dma_pos) {
        uint8_t b = rover_dma_rx_buf[rover_dma_rx_last_pos];
        rover_dma_rx_last_pos = (uint16_t)((rover_dma_rx_last_pos + 1U)
                % ROVER_DMA_RX_BUF_SIZE);
        Rover_FeedByte(b);
    }
}

/* Like HAL_Delay, but keeps the rover DMA RX buffer drained the whole time
 * so a long wait can't silently overrun the 512-byte circular buffer (which
 * fills in ~5 ms at 921600 baud). */
static void Rover_DrainingDelay(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        Rover_DrainDmaRx();
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ICACHE_Init();
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
#ifdef GPS_HARD_RESET_AND_HALT
  HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_RESET);
  /* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_RESET); */ /* PB1 trace cut, pin is analog */
  HAL_Delay(150U);

  HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_SET);
  /* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET); */ /* PB1 trace cut, pin is analog */

  while (1) {
    gps_mode_pin_state = (uint8_t)HAL_GPIO_ReadPin(BASE_RST_GPIO_Port,
        BASE_RST_Pin);
  }
#elif defined(GPS_HARD_RESET_AND_RESTOREPAR)
  HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_RESET);
  /* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_RESET); */ /* PB1 trace cut, pin is analog */
  HAL_Delay(150U);

  HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_SET);
  /* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET); */ /* PB1 trace cut, pin is analog */
  HAL_Delay(100U);

  static const char restore_cmd[] = "$PQTMRESTOREPAR*13\r\n";
	static const char rover_save_cmd[] = "$PQTMSAVEPAR*5A\r\n";

  HAL_UART_Transmit(&huart3, (uint8_t *)restore_cmd, (uint16_t)(sizeof(restore_cmd) - 1U), 100U);

  HAL_Delay(100);
  HAL_UART_Transmit(&huart3, (uint8_t *)rover_save_cmd, (uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);

  while (1) {
    gps_mode_pin_state = (uint8_t)HAL_GPIO_ReadPin(BASE_RST_GPIO_Port, BASE_RST_Pin);
  }
#elif defined(GPS_HARD_RESET_AND_PMTK104)
  /* PMTK/PAIR-family factory reset variant. $PMTK104 is MTK's "full cold start"
   * which also wipes user-saved config. Use this if PQTMRESTOREPAR doesn't take.
   * NOTE: after a full factory reset the module's UART baud reverts to factory
   * default (typically 460800). If you're at 921600 you'll need to either send
   * the command at both baud rates or accept losing comms until reprogramming. */
  HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_RESET);
  /* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_RESET); */ /* PB1 trace cut, pin is analog */
  HAL_Delay(150U);

  HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_SET);
  /* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET); */ /* PB1 trace cut, pin is analog */
  HAL_Delay(100U);

  static const char pmtk_full_cold[] = "$PMTK104*37\r\n";
  static const char pmtk_save_cmd[]  = "$PQTMSAVEPAR*5A\r\n";

  /* Fire at current baud (921600) */
  HAL_UART_Transmit(&huart3, (uint8_t *)pmtk_full_cold,
      (uint16_t)(sizeof(pmtk_full_cold) - 1U), 100U);
  HAL_Delay(100U);

  /* Then re-init huart3 at factory 460800 and resend, in case the module already
   * sits at factory baud or jumps there mid-command. */
  huart3.Init.BaudRate = 460800U;
  (void)HAL_UART_Init(&huart3);
  HAL_UART_Transmit(&huart3, (uint8_t *)pmtk_full_cold,
      (uint16_t)(sizeof(pmtk_full_cold) - 1U), 100U);
  HAL_Delay(100U);
  HAL_UART_Transmit(&huart3, (uint8_t *)pmtk_save_cmd,
      (uint16_t)(sizeof(pmtk_save_cmd) - 1U), 100U);

  while (1) {
    gps_mode_pin_state = (uint8_t)HAL_GPIO_ReadPin(BASE_RST_GPIO_Port, BASE_RST_Pin);
  }
#else
#ifdef ROVER_BAUD_PROGRAM
	/* Step 1: All peripherals initialised above.
     Step 2: BASE_RST is LOW (held in reset by GPIO init).
             ROVER_RST is HIGH (rover running at factory 460800).
     Allow rover to settle before sending commands. */
	HAL_Delay(500U);

	/* Step 3: Program rover UART baud and enable NMEA messages */


//	static const char rover_baud_cmd[] = "$PQTMCFGUART,W,460800*13\r\n";

//	    static const char rover_baud_cmd[] = "$PAIR864,0,0,460800*16\r\n";
	    static const char rover_baud_cmd[] = "$PAIR864,0,0,921600*10\r\n";
	static const char rover_gga_cmd[]  = "$PAIR062,0,1*3F\r\n";
	static const char rover_rmc_cmd[]  = "$PAIR062,4,1*3B\r\n";

	static const char rover_gga_dis_cmd[]  = "$PAIR062,0,0*3E\r\n";
	static const char rover_rmc_dis_cmd[]  = "$PAIR062,4,0*3A\r\n";
	static const char rover_save_cmd[] = "$PQTMSAVEPAR*5A\r\n";
	static const char restore_cmd[] = "$PQTMRESTOREPAR*13\r\n";
	static const char bs1[] = "$PAIR514*3A\r\n";
	static const char bs2[] = "$PAIR513*3D\r\n";
	static const char bs3[] = "$PAIR865,0,0*31\r\n";
//	static const char en_rmc_cmd[] = "$PQTMCFGMSGRATE,W,RMC,1*17\r\n";
//	static const char en_gga_cmd[] = "$PQTMCFGMSGRATE,W,GGA,1*0A\r\n";

	static const char en_vtg_cmd[] = "$PAIR062,5,1*3F\r\n";
	static const char en_rmc_cmd[] = "$PAIR062,4,1*3E\r\n";
	static const char en_gga_cmd[] = "$PAIR062,0,1*3F\r\n";

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
	HAL_UART_Transmit(&huart3, (uint8_t *)restore_cmd,
			(uint16_t)(sizeof(restore_cmd) - 1U), 100U);
	HAL_Delay(1000U);
	HAL_UART_Transmit(&huart3, (uint8_t *)rover_save_cmd,
			(uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
	HAL_Delay(1000U);
	HAL_UART_Transmit(&huart3, (uint8_t *)bs1,
			(uint16_t)(sizeof(bs1) - 1U), 100U);
	HAL_Delay(1000U);
	HAL_UART_Transmit(&huart3, (uint8_t *)bs2,
			(uint16_t)(sizeof(bs2) - 1U), 100U);
	HAL_Delay(1000U);
	HAL_UART_Transmit(&huart3, (uint8_t *)bs3,
			(uint16_t)(sizeof(bs3) - 1U), 100U);
	HAL_Delay(1000U);

	/* 3a: Send baud-change command at factory baud (460800) */
//	huart3.Init.BaudRate = 460800U;
//				(void)HAL_UART_Init(&huart3);
//			for (i = 0U; i < 5U; i++)
//			{
//				HAL_UART_Transmit(&huart3, (uint8_t *)rover_baud_cmd,
//						(uint16_t)(sizeof(rover_baud_cmd) - 1U), 100U);
//				HAL_Delay(1000U);
//			}
//			huart3.Init.BaudRate = 921600U;
//			HAL_UART_Transmit(&huart3, (uint8_t *)en_gga_cmd,
//									(uint16_t)(sizeof(en_gga_cmd) - 1U), 100U);
//			HAL_Delay(1000U);
//			HAL_UART_Transmit(&huart3, (uint8_t *)en_rmc_cmd,
//									(uint16_t)(sizeof(en_rmc_cmd) - 1U), 100U);
//			HAL_Delay(1000U);
//			HAL_UART_Transmit(&huart3, (uint8_t *)en_vtg_cmd,
//					(uint16_t)(sizeof(en_vtg_cmd) - 1U), 100U);
//			HAL_Delay(1000U);
//
//			HAL_UART_Transmit(&huart3, (uint8_t *)rover_save_cmd,
//					(uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
//			HAL_Delay(1000U);
//			HAL_UART_Transmit(&huart3, (uint8_t *)rover_save_cmd,
//					(uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
//			HAL_Delay(1000U);
//
//			HAL_UART_Transmit(&huart3, (uint8_t *)rover_save_cmd,
//					(uint16_t)(sizeof(rover_save_cmd) - 1U), 100U);
//			HAL_Delay(1000U);



//
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
			/* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_RESET); */ /* PB1 trace cut, pin is analog */
			HAL_Delay(150U);

			/* Step 5: Release BASE and ROVER reset simultaneously */
			HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_SET);
			/* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET); */ /* PB1 trace cut, pin is analog */

			/* G for both modules to complete boot */
			HAL_Delay(5000U);

#else /* !ROVER_BAUD_PROGRAM */
			/* Rover already at 921600 — just release both resets */
			HAL_GPIO_WritePin(BASE_RST_GPIO_Port, BASE_RST_Pin, GPIO_PIN_SET);
			/* HAL_GPIO_WritePin(ROVER_RST_GPIO_Port, ROVER_RST_Pin, GPIO_PIN_SET); */ /* PB1 trace cut, pin is analog */
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

			/* Rover (USART1) - passive monitor only.
			 *
			 * The base GPS UART2 TX is wired to the same line as rover RX1 and
			 * STM32 PB6 (USART1 TX). The base firmware already drives RTCM
			 * corrections to the rover on that line; sending additional commands
			 * from the STM32 causes bus contention. So here we only arm
			 * circular DMA RX on USART1 so PB7 (rover TX1 / base RX2) is
			 * captured to the debug stream. The MCU never transmits on PB6. */
			{
				/* Arm DMA RX; do not transmit anything on huart1. */
				__HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF
						| UART_CLEAR_NEF | UART_CLEAR_PEF);
				huart1.ErrorCode = HAL_UART_ERROR_NONE;
				rover_dma_rx_last_pos = 0U;
				(void)HAL_UART_Receive_DMA(&huart1, rover_dma_rx_buf, ROVER_DMA_RX_BUF_SIZE);
			}

//			if (GPS_StartReceiveIT() == HAL_OK) {
//				gps_rx_start_ok = 1U;
//			} else {
//				gps_rx_start_ok = 0U;
//			}

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
#endif /* GPS_HARD_RESET_AND_HALT */

			/* One-shot huart1 register dump so the host log shows the actual
			 * peripheral state after rover programming finished. Reveals RE,
			 * FIFO, BRR, and stuck ISR error flags at a glance. */
			{
				char dbg[160];
				int n = snprintf(dbg, sizeof(dbg),
					"\r\nU1_DIAG CR1=%08lX CR2=%08lX CR3=%08lX BRR=%08lX ISR=%08lX PB7_IDR=%u BR=%lu\r\n",
					(unsigned long)huart1.Instance->CR1,
					(unsigned long)huart1.Instance->CR2,
					(unsigned long)huart1.Instance->CR3,
					(unsigned long)huart1.Instance->BRR,
					(unsigned long)huart1.Instance->ISR,
					(unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7),
					(unsigned long)huart1.Init.BaudRate);
				if (n > 0) {
					GPS_DebugMirror((const uint8_t *)dbg, (uint16_t)n);
				}
			}

#ifdef GPS_PROGRAM_RCVR_MODES
			/* One-shot receiver-mode programming.
			 *   USART3 device  → mode 1 (RTK Base Station, emits RTCM3)
			 *   USART1 device  → mode 2 (Moving-Baseline Rover, emits PQTMTAR/PQTMVEHATT)
			 * Runs AFTER normal init so BASE_RST has been released and DMA RX
			 * is armed on huart1 (otherwise the rover's reboot burst would
			 * overrun the UART and CR3.DMAR never gets set).
			 * After a successful flash + observe of acks in the log, comment
			 * out the define near the top of this file and reflash. Settings
			 * persist on each module's internal flash via PQTMSAVEPAR. */
			{
				static const char set_mode_base [] = "$PQTMCFGRCVRMODE,W,1*2A\r\n";
				static const char set_mode_rover[] = "$PQTMCFGRCVRMODE,W,2*29\r\n";
				static const char savepar       [] = "$PQTMSAVEPAR*5A\r\n";
				static const char srr           [] = "$PQTMSRR*4B\r\n";

				/* USART3 → base (mode 1) */
				HAL_UART_Transmit(&huart3, (uint8_t *)set_mode_base,
					(uint16_t)(sizeof(set_mode_base) - 1U), 100U);
				Rover_DrainingDelay(300U);
				HAL_UART_Transmit(&huart3, (uint8_t *)savepar,
					(uint16_t)(sizeof(savepar) - 1U), 100U);
				Rover_DrainingDelay(300U);

				/* USART1 → rover (mode 2) */
				HAL_UART_Transmit(&huart1, (uint8_t *)set_mode_rover,
					(uint16_t)(sizeof(set_mode_rover) - 1U), 100U);
				Rover_DrainingDelay(300U);
				HAL_UART_Transmit(&huart1, (uint8_t *)savepar,
					(uint16_t)(sizeof(savepar) - 1U), 100U);
				Rover_DrainingDelay(300U);

				/* Soft-reset both so the new mode is loaded fresh. DMA RX is
				 * already armed on huart1, so the rover's reboot burst gets
				 * captured cleanly. */
				HAL_UART_Transmit(&huart3, (uint8_t *)srr,
					(uint16_t)(sizeof(srr) - 1U), 100U);
				HAL_UART_Transmit(&huart1, (uint8_t *)srr,
					(uint16_t)(sizeof(srr) - 1U), 100U);

				Rover_DrainingDelay(2500U);
			}
#endif /* GPS_PROGRAM_RCVR_MODES */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			while (1) {
				uint32_t now = HAL_GetTick();

				gps_mode_pin_state = (uint8_t) HAL_GPIO_ReadPin(BASE_RST_GPIO_Port,
						BASE_RST_Pin);

				PPS_Process(now); // PPS needs to be polled immediately after getting 'now' before GPS handling, otherwise PPS read does not update
				GPS_Process();

				/* Rover (huart1) RX mirror \u2014 drain RXNE byte-by-byte into the
				 * USART2 debug stream so the host log shows whatever the rover
				 * is emitting. Each new line gets an "R> " prefix so it's
				 * trivially distinguishable from base traffic. Non-blocking
				 * flag poll; no impact on timing budget. */
				/* Rover (USART1) DMA RX drain - forward new bytes from the
				 * circular DMA buffer to the USART2 debug stream, prefixing
				 * each new line with "R> " so rover traffic is distinguishable
				 * from base traffic. Handles DMA wrap-around correctly. */
#ifndef DEBUG_MIRROR_BASE_ONLY
				/* Feed each new byte from the rover DMA buffer through the
				 * RTCM3 frame detector.  Complete, CRC-valid frames are
				 * reported as a single ASCII line ("R3> 1074 127B") so no
				 * 0x0A/0x0D bytes inside RTCM binary ever reach USART2. */
				Rover_DrainDmaRx();
#endif /* DEBUG_MIRROR_BASE_ONLY */

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
  huart1.Init.BaudRate = 921600;
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
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  HAL_GPIO_WritePin(BASE_WKUP_GPIO_Port, BASE_WKUP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  /* HAL_GPIO_WritePin(ROVER_WKUP_GPIO_Port, ROVER_WKUP_Pin, GPIO_PIN_SET); */ /* rover wakeup unused */

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /*Configure GPIO pin : BASE_WKUP_Pin */
  GPIO_InitStruct.Pin = BASE_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BASE_WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ROVER_WKUP_Pin */
  /* rover wakeup unused
  GPIO_InitStruct.Pin = ROVER_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROVER_WKUP_GPIO_Port, &GPIO_InitStruct);
  */

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
