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

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef GPS_StartReceiveIT(void);
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void GPS_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
