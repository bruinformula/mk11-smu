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
    float    latitude_deg;
    float    longitude_deg;
    float    altitude_m;
    float    speed_kph;
    float    course_deg;
    float    heading_deg;
    float    hdop;
    /* PQTMTAR (moving-base attitude) */
    uint8_t  tar_quality;
    uint8_t  tar_satellites;
    float    baseline_length_m;
    float    pitch_deg;
    float    heading_acc_deg;
    float    pitch_acc_deg;
    uint32_t last_update_ms;
} GPS_Data_t;

typedef struct {
    uint32_t rx_bytes;
    uint32_t sentences;
    uint32_t rmc_count;
    uint32_t gga_count;
    uint32_t vtg_count;
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
