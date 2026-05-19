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
	float latitude_deg;
	float longitude_deg;
	float speed_knots;
	float speed_kph;
	float course_deg;
	float altitude_m;
	float hdop;
	char utc_time[16];
	char utc_date[16];
} GPS_Data_t;

extern volatile GPS_Data_t gps_data;
extern volatile uint32_t gps_rx_count;
extern volatile uint8_t gps_last_byte;
extern volatile char gps_last_sentence[GPS_LINE_BUFFER_SIZE];
extern volatile uint8_t gps_sentence_ready;
extern volatile uint32_t gps_sentence_count;
extern volatile uint32_t gps_rmc_count;
extern volatile uint32_t gps_gga_count;
extern volatile uint32_t gps_vtg_count;

/* Binary-tolerant parser debug counters */
extern volatile uint32_t gps_dollar_count;
extern volatile uint32_t gps_ascii_sentence_count;
extern volatile uint32_t gps_binary_drop_count;
extern volatile uint32_t gps_nonascii_sentence_drop_count;
extern volatile uint32_t gps_line_overflow_count;

/* PQTMTAR / moving-base heading debug */
extern volatile uint32_t gps_pqtmtar_count;
extern volatile uint8_t gps_pqtmtar_status;
extern volatile float gps_pqtmtar_heading_deg;
extern volatile float gps_pqtmtar_pitch_deg;
extern volatile float gps_pqtmtar_roll_deg;
extern volatile float gps_pqtmtar_baseline_m;
extern volatile uint8_t gps_pqtmtar_msgver;
extern volatile float gps_pqtmtar_utc_time;
extern volatile uint8_t gps_pqtmtar_quality;
extern volatile float gps_pqtmtar_pitch_accuracy_deg;
extern volatile float gps_pqtmtar_heading_accuracy_deg;
extern volatile uint8_t gps_pqtmtar_used_sv;

/* PQTMTAR solution-valid debug */
extern volatile uint8_t gps_pqtmtar_solution_valid;
extern volatile uint32_t gps_pqtmtar_valid_count;
extern volatile uint32_t gps_pqtmtar_invalid_count;
extern volatile uint32_t gps_pqtmtar_last_valid_ms;
extern volatile uint32_t gps_pqtmtar_last_invalid_ms;

/* PQTM command response debug */
extern volatile char gps_last_pqtm_response[GPS_LINE_BUFFER_SIZE];

extern volatile uint32_t gps_pqtmcfgprot_ok_count;
extern volatile uint32_t gps_pqtmcfgprot_error_count;

extern volatile uint32_t gps_pqtmcfgmsgrate_ok_count;
extern volatile uint32_t gps_pqtmcfgmsgrate_error_count;

extern volatile uint32_t gps_pqtmsavepar_ok_count;
extern volatile uint32_t gps_pqtmsavepar_error_count;

extern volatile uint32_t gps_pqtm_response_other_count;

/* Raw fields for debugging until final field meanings are confirmed */
extern volatile float gps_pqtmtar_f1;
extern volatile float gps_pqtmtar_f2;
extern volatile float gps_pqtmtar_f3;
extern volatile float gps_pqtmtar_f4;
extern volatile float gps_pqtmtar_f5;
extern volatile float gps_pqtmtar_f6;
extern volatile float gps_pqtmtar_f7;
extern volatile float gps_pqtmtar_f8;

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef GPS_StartReceiveIT(void);
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void GPS_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
