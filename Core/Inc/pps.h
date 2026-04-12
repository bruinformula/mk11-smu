#ifndef PPS_H
#define PPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

extern volatile uint8_t gps1_pps_pin_state;
extern volatile uint32_t gps1_pps_poll_rise_count;
extern volatile uint32_t gps1_pps_fall_count;
extern volatile uint32_t gps1_pps_count;
extern volatile uint32_t gps1_last_pps_ms;

HAL_StatusTypeDef PPS_Init(void);
void PPS_Process(uint32_t now_ms);
void PPS_ForcePinConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* PPS_H */
