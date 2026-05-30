#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define GPS_RX_BUFFER_SIZE    4096U
#define GPS_LINE_BUFFER_SIZE  128U

typedef struct {
	uint8_t fix_valid;
	uint8_t fix_quality;
	uint8_t satellites;
	uint8_t heading_valid;
	uint8_t heading_quality;
	float latitude_deg;
	float longitude_deg;
	float speed_knots;
	float speed_kph;
	float course_deg;
	float heading_deg;
	float heading_accuracy_deg;
	float baseline_length_m;
	float pitch_deg;
	float altitude_m;
	float hdop;
	char utc_time[16];
	char utc_date[16];
} GPS_Data_t;

typedef struct {
	uint32_t rx_count;
	uint8_t last_byte;
	char last_sentence[GPS_LINE_BUFFER_SIZE];
	uint8_t sentence_ready;
	uint32_t sentence_count;
	uint32_t rmc_count;
	uint32_t gga_count;
	uint32_t vtg_count;
	uint32_t pqtmtar_count;
	uint32_t uart_irq_count;
	uint32_t uart_last_isr;
	uint32_t uart_last_error_code;
	uint8_t start_receive_status;
	uint8_t uart_rx_state_after_start;
	uint32_t init_count;
	uint32_t start_receive_calls;
	uint32_t uart_cr1_after_start;
	uint32_t uart_cr3_after_start;
	uint32_t transport_preflight_mask;
	uint32_t uart_poll_rxne_count;
	uint32_t uart_poll_error_count;
	uint32_t active_baud_rate;
	uint32_t detected_baud_rate;
	uint32_t baud_switch_count;
	uint8_t baud_locked;
	uint8_t transport_preflight_ok;
	uint8_t baud_candidate_index;
	uint32_t config_command_count;
	uint8_t config_command_status;
	uint32_t config_readback_count;
	uint8_t config_readback_status;
	uint8_t heading_message_enabled;
	uint8_t heading_message_verified;
	uint32_t debug_dump_count;
	uint32_t debug_dump_fail_count;
	uint8_t uart_rx_pin_level;
	uint8_t uart_rx_pin_last_level;
	uint32_t uart_rx_pin_transition_count;
	uint32_t pair001_count;
	int8_t pair001_last_result;
	char pair001_last_msgid[8];
	uint8_t gga_enable_acked;
} GPS_Diag_t;

extern volatile GPS_Data_t gps_data;
extern volatile GPS_Diag_t gps_diag;
extern volatile uint32_t gps_init_stage_debug;
extern volatile uint32_t gps_init_retry_count_debug;

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef GPS_StartReceiveIT(void);
void GPS_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void GPS_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void GPS_Process(void);

/* Public USART2 debug mirror — forwards arbitrary bytes into the same queued
 * USART2 stream the GPS driver uses, so callers outside gps.c can interleave
 * diagnostic data without racing the DMA TX path. */
void GPS_DebugMirror(const uint8_t *data, uint16_t len);

/* --- Debug mirror mode ---------------------------------------------------
 * Uncomment exactly ONE of the two defines below to restrict what is
 * forwarded to the USART2 debug stream.  Leave both commented to mirror
 * both USART3 (base GPS) and USART1 (rover) simultaneously (default).
 *
 *   DEBUG_MIRROR_BASE_ONLY   — only USART3 raw bytes reach USART2;
 *                              the rover R> drain in the main loop is
 *                              suppressed.
 *
 *   DEBUG_MIRROR_ROVER_ONLY  — only the USART1 rover R> stream reaches
 *                              USART2; the USART3 raw-mirror inside
 *                              GPS_DrainRxDma is suppressed (parsed
 *                              sentence fields still update gps_data).
 */
/* #define DEBUG_MIRROR_BASE_ONLY  */
 #define DEBUG_MIRROR_ROVER_ONLY 

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
