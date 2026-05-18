#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define GPS_RX_BUFFER_SIZE    512U
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
	uint32_t uart_poll_rxne_count;
	uint32_t uart_poll_error_count;
	uint32_t active_baud_rate;
	uint32_t detected_baud_rate;
	uint32_t baud_switch_count;
	uint8_t baud_locked;
	uint8_t baud_candidate_index;
	uint32_t config_command_count;
	uint8_t config_command_status;
	uint8_t heading_message_enabled;
	uint8_t uart_rx_pin_level;
	uint8_t uart_rx_pin_last_level;
	uint32_t uart_rx_pin_transition_count;
} GPS_Diag_t;

extern volatile GPS_Data_t gps_data;
extern volatile GPS_Diag_t gps_diag;

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef GPS_StartReceiveIT(void);
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void GPS_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
