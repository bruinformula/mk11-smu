#include "pps.h"

volatile uint8_t gps1_pps_pin_state = 0U;
volatile uint32_t gps1_pps_poll_rise_count = 0U;
volatile uint32_t gps1_pps_fall_count = 0U;
volatile uint32_t gps1_pps_count = 0U;
volatile uint32_t gps1_last_pps_ms = 0U;

static uint8_t pps_prev_state = 1U;
static uint8_t pps_initialized = 0U;

void PPS_ForcePinConfig(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	if (GPS_1PPS_GPIO_Port == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (GPS_1PPS_GPIO_Port == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	} else if (GPS_1PPS_GPIO_Port == GPIOC) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}

	GPIO_InitStruct.Pin = GPS_1PPS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPS_1PPS_GPIO_Port, &GPIO_InitStruct);
}

HAL_StatusTypeDef PPS_Init(void) {
	PPS_ForcePinConfig();

	gps1_pps_pin_state = 0U;
	gps1_pps_poll_rise_count = 0U;
	gps1_pps_fall_count = 0U;
	gps1_pps_count = 0U;
	gps1_last_pps_ms = 0U;

	pps_prev_state = 1U;
	pps_initialized = 0U;

	return HAL_OK;
}

void PPS_Process(uint32_t now_ms) {
	uint8_t curr_state;

	curr_state = (uint8_t) HAL_GPIO_ReadPin(GPS_1PPS_GPIO_Port, GPS_1PPS_Pin);
	gps1_pps_pin_state = curr_state;

	if (pps_initialized == 0U) {
		pps_prev_state = curr_state;
		pps_initialized = 1U;
		return;
	}

	if ((pps_prev_state == 0U) && (curr_state == 1U)) {
		gps1_pps_poll_rise_count++;
	}

	if ((pps_prev_state == 1U) && (curr_state == 0U)) {
		gps1_pps_fall_count++;
		gps1_pps_count++;
		gps1_last_pps_ms = now_ms;
	}

	pps_prev_state = curr_state;
}
