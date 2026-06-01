#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct {
    uint8_t  fix_valid;
    uint8_t  fix_quality;
    uint8_t  satellites;
    uint8_t  heading_valid;
    uint8_t  heading_quality;
    float    latitude_deg;
    float    longitude_deg;
    float    altitude_m;
    float    speed_kph;
    float    course_deg;
    float    heading_deg;
    float    hdop;
    float    heading_accuracy_deg;
    float    baseline_length_m;
    float    pitch_deg;
    char     utc_time[16];
    char     utc_date[16];
} GPS_Data_t;

typedef struct {
    uint32_t rx_bytes;
    uint32_t sentences;
    uint32_t rmc_count;
    uint32_t gga_count;
    uint32_t vtg_count;
    uint32_t pqtmtar_count;
    uint32_t sentence_count;
    uint32_t uart_last_error_code;
    uint8_t  start_status;
    uint8_t  last_byte;
} GPS_Diag_t;

extern volatile GPS_Data_t gps_data;
extern volatile GPS_Diag_t gps_diag;

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart);
void GPS_Process(void);
void GPS_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void GPS_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void GPS_DebugMirror(const uint8_t *data, uint16_t len);
void GPS_ErrorRestart(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
