#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GPS_DEFAULT_BAUD_RATE       460800U
#define GPS_PQTMTAR_ENABLE_CMD      "$PQTMCFGMSGRATE,W,PQTMTAR,1,1*09\r\n"
#define GPS_PQTMTAR_QUERY_CMD       "$PQTMCFGMSGRATE,R,PQTMTAR,1*11\r\n"
#define GPS_PAIR_INIT_CMD           "$PAIR002*38\r\n"
#define GPS_QUERY_VER_CMD           "$PQTMQVER*08\r\n"
#define GPS_ENABLE_GGA_CMD          "$PQTMCFGMSGRATE,W,GGA,1*0A\r\n"
#define GPS_ENABLE_RMC_CMD          "$PQTMCFGMSGRATE,W,RMC,1*17\r\n"
#define GPS_EN_VTG_PAIR_CMD 		"$PAIR062,5,1*3A\r\n"
#define GPS_EN_RMC_PAIR_CMD			"$PAIR062,4,1*3B\r\n"
#define GPS_EN_GGA_PAIR_CMD			"$PAIR062,0,1*3F\r\n"
#define GPS_DISABLE_GGA_CMD          "$PQTMCFGMSGRATE,W,GGA,0*08\r\n"
#define GPS_DISABLE_RMC_CMD          "$PQTMCFGMSGRATE,W,RMC,0*16\r\n"
#define GPS_SAVEPAR_CMD             "$PQTMSAVEPAR*5A\r\n"
#define GPS_COMMAND_RETRY_MS        1000U
#define GPS_CONFIG_RETRY_MS         5000U
#define GPS_CONFIG_MAX_RETRIES      3U
#define GPS_DEBUG                    1U
#define GPS_DMA_RX_BUFFER_SIZE       2048U
#define GPS_TX_BUFFER_SIZE     128U
#define GPS_UART2_TX_BUFFER_SIZE     256U
#define GPS_UART2_TX_QUEUE_SIZE      2048U

typedef enum
{
	GPS_CONFIG_STAGE_ENABLE_WRITE = 0,
	GPS_CONFIG_STAGE_WAIT_ENABLE_ACK,
	GPS_CONFIG_STAGE_QUERY_READBACK,
	GPS_CONFIG_STAGE_WAIT_READBACK,
	GPS_CONFIG_STAGE_VERIFIED
} GPS_ConfigStage_t;

static UART_HandleTypeDef *gps_uart = NULL;
static volatile uint16_t gps_rx_head = 0U;
static volatile uint16_t gps_rx_tail = 0U;
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static uint8_t gps_dma_rx_buffer[GPS_DMA_RX_BUFFER_SIZE];
static volatile uint16_t gps_dma_rx_last_pos = 0U;
static uint8_t gps_tx_buffer[GPS_TX_BUFFER_SIZE];
static uint8_t gps_uart2_tx_buffer[GPS_UART2_TX_BUFFER_SIZE];
static uint8_t gps_uart2_tx_queue[GPS_UART2_TX_QUEUE_SIZE];
static volatile uint8_t gps_tx_busy = 0U;
static volatile uint8_t gps_uart2_tx_busy = 0U;
static volatile uint16_t gps_uart2_tx_head = 0U;
static volatile uint16_t gps_uart2_tx_tail = 0U;

static char gps_line_buffer[GPS_LINE_BUFFER_SIZE];
static uint16_t gps_line_index = 0U;
static GPS_ConfigStage_t gps_config_stage = GPS_CONFIG_STAGE_ENABLE_WRITE;
static uint32_t gps_last_command_ms = 0U;
static uint8_t gps_config_retry_count = 0U;
static uint8_t gps_config_retry_exhausted = 0U;
volatile char LIVE_CMD[GPS_LINE_BUFFER_SIZE] = {0};
volatile char LIVE_CMD_TEXT[GPS_LINE_BUFFER_SIZE] = {0};
static char gps_last_live_cmd[GPS_LINE_BUFFER_SIZE] = {0};
static char gps_last_live_cmd_text[GPS_LINE_BUFFER_SIZE] = {0};

volatile GPS_Data_t gps_data = {0};
volatile GPS_Diag_t gps_diag = {0};

extern UART_HandleTypeDef huart2;

static void GPS_HandleSentence(const char *sentence);
static void GPS_ProcessPendingBytes(void);
static void GPS_ProcessLiveCommandText(void);
static void GPS_ProcessLiveCommand(void);
static void GPS_MirrorCommandToUart2(const char *command);
static void GPS_MirrorSentenceToUart2(const char *sentence);
static void GPS_MirrorDataCsvToUart2(void);
static void GPS_BuildCommandFromText(const char *source, char *dest, size_t dest_size);
static void GPS_PushByte(uint8_t byte);
static uint8_t GPS_ReadActiveRxPinLevel(void);
static HAL_StatusTypeDef GPS_StartReceiveDma(void);
static void GPS_DrainRxDma(void);
static void GPS_QueueMirrorBytesToUart2(const uint8_t *data, uint16_t data_len);
static void GPS_KickUart2Tx(void);
static HAL_StatusTypeDef GPS_TransmitDmaSync(UART_HandleTypeDef *uart,
		volatile uint8_t *busy_flag,
		uint8_t *tx_buffer,
		uint16_t tx_buffer_size,
		const uint8_t *data,
		uint16_t data_len,
		uint32_t timeout_ms);

static size_t GPS_BoundedStringLength(const char *text, size_t limit)
{
	size_t length = 0U;

	if (text == NULL)
	{
		return 0U;
	}

	while ((length < limit) && (text[length] != '\0'))
	{
		length++;
	}

	return length;
}

static void GPS_BuildCommandFromText(const char *source, char *dest, size_t dest_size)
{
	size_t source_len;
	size_t payload_len;

	if ((source == NULL) || (dest == NULL) || (dest_size == 0U))
	{
		return;
	}

	dest[0] = '\0';

	if (dest_size < 3U)
	{
		return;
	}

	source_len = GPS_BoundedStringLength(source, GPS_LINE_BUFFER_SIZE - 1U);
	payload_len = source_len;

	while ((payload_len > 0U) && ((source[payload_len - 1U] == '\r') || (source[payload_len - 1U] == '\n')))
	{
		payload_len--;
	}

	if (payload_len == 0U)
	{
		return;
	}

	if (payload_len > (dest_size - 3U))
	{
		payload_len = dest_size - 3U;
	}

	memcpy(dest, source, payload_len);
	dest[payload_len] = '\r';
	dest[payload_len + 1U] = '\n';
	dest[payload_len + 2U] = '\0';
}

static void GPS_AllowImmediateRetry(void)
{
	gps_last_command_ms = HAL_GetTick() - GPS_CONFIG_RETRY_MS;
}

static uint16_t GPS_NextIndex(uint16_t idx)
{
	return (uint16_t)((idx + 1U) % GPS_RX_BUFFER_SIZE);
}

static uint16_t GPS_NextUart2TxIndex(uint16_t idx)
{
	return (uint16_t)((idx + 1U) % GPS_UART2_TX_QUEUE_SIZE);
}

static void GPS_UpdateUartDiagState(void)
{
	if (gps_uart == NULL)
	{
		gps_diag.uart_rx_state_after_start = 0U;
		gps_diag.uart_cr1_after_start = 0U;
		gps_diag.uart_cr3_after_start = 0U;
		gps_diag.uart_last_isr = 0U;
		gps_diag.active_baud_rate = 0U;
		return;
	}

	gps_diag.uart_rx_state_after_start = (uint8_t)gps_uart->RxState;
	gps_diag.uart_cr1_after_start = gps_uart->Instance->CR1;
	gps_diag.uart_cr3_after_start = gps_uart->Instance->CR3;
	gps_diag.uart_last_isr = gps_uart->Instance->ISR;
	gps_diag.active_baud_rate = gps_uart->Init.BaudRate;
}

static uint8_t GPS_ReadActiveRxPinLevel(void)
{
	GPIO_TypeDef *port = GPIOB;
	uint16_t pin = GPIO_PIN_7;

	if (gps_uart != NULL)
	{
		if (gps_uart->Instance == USART3)
		{
			pin = GPIO_PIN_11;
		}
		else if (gps_uart->Instance == USART1)
		{
			pin = GPIO_PIN_7;
		}
	}

	return (uint8_t)(((port->IDR & pin) != 0U) ? 1U : 0U);
}

static void GPS_SampleRxPinLevel(void)
{
	uint8_t level = GPS_ReadActiveRxPinLevel();

	gps_diag.uart_rx_pin_level = level;

	if (level != gps_diag.uart_rx_pin_last_level)
	{
		gps_diag.uart_rx_pin_transition_count++;
		gps_diag.uart_rx_pin_last_level = level;
	}
}

static void GPS_ResetRxState(void)
{
	gps_rx_head = 0U;
	gps_rx_tail = 0U;
	gps_line_index = 0U;
	gps_dma_rx_last_pos = 0U;
	gps_tx_busy = 0U;
	gps_uart2_tx_busy = 0U;
	gps_uart2_tx_head = 0U;
	gps_uart2_tx_tail = 0U;
	gps_diag.sentence_ready = 0U;
	memset(gps_dma_rx_buffer, 0, sizeof(gps_dma_rx_buffer));
	memset(gps_uart2_tx_queue, 0, sizeof(gps_uart2_tx_queue));
	memset((void *)gps_diag.last_sentence, 0, sizeof(gps_diag.last_sentence));
}

static HAL_StatusTypeDef GPS_StartReceiveDma(void)
{
	HAL_StatusTypeDef status;

	if (gps_uart == NULL)
	{
		gps_diag.start_receive_status = (uint8_t)HAL_ERROR;
		GPS_UpdateUartDiagState();
		return HAL_ERROR;
	}

	status = HAL_UART_Receive_DMA(gps_uart, gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);

	if ((status == HAL_BUSY)
			&& ((gps_uart->RxState == HAL_UART_STATE_BUSY_RX)
					|| (gps_uart->RxState == HAL_UART_STATE_BUSY_TX_RX)))
	{
		status = HAL_OK;
	}

	gps_diag.start_receive_status = (uint8_t)status;
	gps_diag.uart_last_error_code = gps_uart->ErrorCode;
	GPS_UpdateUartDiagState();

	return status;
}

static void GPS_DrainRxDma(void)
{
	uint16_t dma_pos;

	if ((gps_uart == NULL) || (gps_uart->hdmarx == NULL))
	{
		return;
	}

	dma_pos = (uint16_t)(GPS_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps_uart->hdmarx));

	if (dma_pos >= GPS_DMA_RX_BUFFER_SIZE)
	{
		dma_pos = 0U;
	}

	if (dma_pos == gps_dma_rx_last_pos)
	{
		return;
	}

	if (dma_pos > gps_dma_rx_last_pos)
	{
		while (gps_dma_rx_last_pos < dma_pos)
		{
			GPS_PushByte(gps_dma_rx_buffer[gps_dma_rx_last_pos]);
			gps_dma_rx_last_pos++;
		}
	}
	else
	{
		while (gps_dma_rx_last_pos < GPS_DMA_RX_BUFFER_SIZE)
		{
			GPS_PushByte(gps_dma_rx_buffer[gps_dma_rx_last_pos]);
			gps_dma_rx_last_pos++;
		}

		gps_dma_rx_last_pos = 0U;

		while (gps_dma_rx_last_pos < dma_pos)
		{
			GPS_PushByte(gps_dma_rx_buffer[gps_dma_rx_last_pos]);
			gps_dma_rx_last_pos++;
		}
	}

	gps_diag.uart_poll_rxne_count++;
}

static void GPS_QueueMirrorBytesToUart2(const uint8_t *data, uint16_t data_len)
{
	uint16_t idx;
	uint32_t primask;
	uint8_t dropped = 0U;

	if ((data == NULL) || (data_len == 0U) || (huart2.hdmatx == NULL))
	{
		return;
	}

	primask = __get_PRIMASK();
	__disable_irq();

	for (idx = 0U; idx < data_len; idx++)
	{
		uint16_t next = GPS_NextUart2TxIndex(gps_uart2_tx_head);

		if (next == gps_uart2_tx_tail)
		{
			dropped = 1U;
			break;
		}

		gps_uart2_tx_queue[gps_uart2_tx_head] = data[idx];
		gps_uart2_tx_head = next;
	}

	if (primask == 0U)
	{
		__enable_irq();
	}

	if (dropped != 0U)
	{
		gps_diag.debug_dump_fail_count++;
	}

	GPS_KickUart2Tx();
}

static void GPS_KickUart2Tx(void)
{
	uint16_t tail;
	uint16_t head;
	uint16_t tx_len = 0U;
	uint32_t primask;
	HAL_StatusTypeDef status;

	if (huart2.hdmatx == NULL)
	{
		return;
	}

	primask = __get_PRIMASK();
	__disable_irq();

	if ((gps_uart2_tx_busy != 0U) || (gps_uart2_tx_head == gps_uart2_tx_tail))
	{
		if (primask == 0U)
		{
			__enable_irq();
		}
		return;
	}

	tail = gps_uart2_tx_tail;
	head = gps_uart2_tx_head;

	while ((tail != head) && (tx_len < GPS_UART2_TX_BUFFER_SIZE))
	{
		gps_uart2_tx_buffer[tx_len++] = gps_uart2_tx_queue[tail];
		tail = GPS_NextUart2TxIndex(tail);
	}

	gps_uart2_tx_tail = tail;
	gps_uart2_tx_busy = 1U;

	if (primask == 0U)
	{
		__enable_irq();
	}

	status = HAL_UART_Transmit_DMA(&huart2, gps_uart2_tx_buffer, tx_len);

	if (status != HAL_OK)
	{
		primask = __get_PRIMASK();
		__disable_irq();
		gps_uart2_tx_busy = 0U;
		if (primask == 0U)
		{
			__enable_irq();
		}
		gps_diag.debug_dump_fail_count++;
	}
}

static HAL_StatusTypeDef GPS_TransmitDmaSync(UART_HandleTypeDef *uart,
		volatile uint8_t *busy_flag,
		uint8_t *tx_buffer,
		uint16_t tx_buffer_size,
		const uint8_t *data,
		uint16_t data_len,
		uint32_t timeout_ms)
{
	HAL_StatusTypeDef status;
	uint32_t start_ms;

	if ((uart == NULL) || (busy_flag == NULL) || (tx_buffer == NULL) || (data == NULL))
	{
		return HAL_ERROR;
	}

	if ((data_len == 0U) || (data_len > tx_buffer_size) || (uart->hdmatx == NULL))
	{
		return HAL_ERROR;
	}

	start_ms = HAL_GetTick();
	while (*busy_flag != 0U)
	{
		GPS_DrainRxDma();

		if ((HAL_GetTick() - start_ms) >= timeout_ms)
		{
			return HAL_TIMEOUT;
		}
	}

	memcpy(tx_buffer, data, data_len);
	*busy_flag = 1U;
	status = HAL_UART_Transmit_DMA(uart, tx_buffer, data_len);

	if (status != HAL_OK)
	{
		*busy_flag = 0U;
		return status;
	}

	start_ms = HAL_GetTick();
	while (*busy_flag != 0U)
	{
		GPS_DrainRxDma();

		if ((HAL_GetTick() - start_ms) >= timeout_ms)
		{
			return HAL_TIMEOUT;
		}
	}

	return HAL_OK;
}

static HAL_StatusTypeDef GPS_SendCommand(const char *command)
{
	HAL_StatusTypeDef status;

	if ((gps_uart == NULL) || (command == NULL))
	{
		gps_diag.config_command_status = (uint8_t)HAL_ERROR;
		return HAL_ERROR;
	}

	GPS_MirrorCommandToUart2(command);

	status = GPS_TransmitDmaSync(gps_uart,
			&gps_tx_busy,
			gps_tx_buffer,
			GPS_TX_BUFFER_SIZE,
			(const uint8_t *)command,
			(uint16_t)strlen(command),
			100U);

	gps_diag.config_command_count++;
	gps_diag.config_command_status = (uint8_t)status;
	gps_diag.uart_last_error_code = gps_uart->ErrorCode;
	GPS_UpdateUartDiagState();

	return status;
}

static void GPS_MirrorCommandToUart2(const char *command)
{
#if GPS_DEBUG
uint16_t command_len;
uint8_t echo_buf[72];
uint16_t echo_len;

if ((command == NULL) || (command[0] == '\0'))
{
	return;
}

command_len = (uint16_t)strlen(command);
echo_len = (uint16_t)(4U + command_len);

if (echo_len > sizeof(echo_buf))
{
	echo_len = (uint16_t)sizeof(echo_buf);
}

memcpy(echo_buf, "TX> ", 4U);
memcpy(echo_buf + 4U, command, echo_len - 4U);
GPS_QueueMirrorBytesToUart2(echo_buf, echo_len);
#else
(void)command;
#endif
}

static void GPS_MirrorSentenceToUart2(const char *sentence)
{
#if GPS_DEBUG
	uint16_t sentence_len;
	uint8_t sentence_buf[GPS_LINE_BUFFER_SIZE + 2U];

	if ((sentence == NULL) || (sentence[0] == '\0'))
	{
		return;
	}

	sentence_len = (uint16_t)strlen(sentence);
	if (sentence_len > GPS_LINE_BUFFER_SIZE)
	{
		sentence_len = GPS_LINE_BUFFER_SIZE;
	}

	memcpy(sentence_buf, sentence, sentence_len);
	sentence_buf[sentence_len] = '\r';
	sentence_buf[sentence_len + 1U] = '\n';

	GPS_QueueMirrorBytesToUart2(sentence_buf, (uint16_t)(sentence_len + 2U));
#else
	(void)sentence;
#endif
}

static void GPS_MirrorDataCsvToUart2(void)
{
#if !GPS_DEBUG
	char csv_buf[192];
	int csv_len;

	csv_len = snprintf(csv_buf,
			sizeof(csv_buf),
			"%s,%s,%u,%u,%u,%.7f,%.7f,%.3f,%.3f,%.3f,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
			(const char *)gps_data.utc_time,
			(const char *)gps_data.utc_date,
			(unsigned int)gps_data.fix_valid,
			(unsigned int)gps_data.fix_quality,
			(unsigned int)gps_data.satellites,
			(double)gps_data.latitude_deg,
			(double)gps_data.longitude_deg,
			(double)gps_data.speed_knots,
			(double)gps_data.speed_kph,
			(double)gps_data.course_deg,
			(unsigned int)gps_data.heading_valid,
			(unsigned int)gps_data.heading_quality,
			(double)gps_data.heading_deg,
			(double)gps_data.heading_accuracy_deg,
			(double)gps_data.baseline_length_m,
			(double)gps_data.pitch_deg,
			(double)gps_data.altitude_m);

	if ((csv_len > 0) && ((size_t)csv_len < sizeof(csv_buf)))
	{
		GPS_QueueMirrorBytesToUart2((const uint8_t *)csv_buf, (uint16_t)csv_len);
	}
#endif
}

static void GPS_PushByte(uint8_t byte)
{
	uint16_t next = GPS_NextIndex(gps_rx_head);

	gps_diag.last_byte = byte;
	gps_diag.rx_count++;

	if (next != gps_rx_tail)
	{
		gps_rx_buffer[gps_rx_head] = byte;
		gps_rx_head = next;
	}
}

static uint32_t GPS_SplitFields(char *buffer, char *fields[], uint32_t max_fields)
{
	uint32_t count = 0U;
	char *p = buffer;

	if ((buffer == NULL) || (fields == NULL) || (max_fields == 0U))
	{
		return 0U;
	}

	fields[count++] = p;

	while ((*p != '\0') && (count < max_fields))
	{
		if (*p == '*')
		{
			*p = '\0';
			break;
		}

		if (*p == ',')
		{
			*p = '\0';
			fields[count++] = p + 1;
		}

		p++;
	}

	return count;
}

static float GPS_ParseLatitude(const char *field, const char *ns)
{
	float raw;
	int degrees;
	float minutes;
	float value;

	if ((field == NULL) || (ns == NULL) || (field[0] == '\0') || (ns[0] == '\0'))
	{
		return 0.0f;
	}

	raw = (float)atof(field);
	degrees = (int)(raw / 100.0f);
	minutes = raw - ((float)degrees * 100.0f);
	value = (float)degrees + (minutes / 60.0f);

	if (ns[0] == 'S')
	{
		value = -value;
	}

	return value;
}

static float GPS_ParseLongitude(const char *field, const char *ew)
{
	float raw;
	int degrees;
	float minutes;
	float value;

	if ((field == NULL) || (ew == NULL) || (field[0] == '\0') || (ew[0] == '\0'))
	{
		return 0.0f;
	}

	raw = (float)atof(field);
	degrees = (int)(raw / 100.0f);
	minutes = raw - ((float)degrees * 100.0f);
	value = (float)degrees + (minutes / 60.0f);

	if (ew[0] == 'W')
	{
		value = -value;
	}

	return value;
}

static void GPS_ParseRMC(const char *sentence)
{
	char buffer[GPS_LINE_BUFFER_SIZE];
	char *fields[20] = {0};
	uint32_t field_count;

	snprintf(buffer, sizeof(buffer), "%s", sentence);
	field_count = GPS_SplitFields(buffer, fields, 20U);

	if (field_count < 10U)
	{
		return;
	}

	snprintf((char *)gps_data.utc_time, sizeof(gps_data.utc_time), "%s", fields[1]);
	snprintf((char *)gps_data.utc_date, sizeof(gps_data.utc_date), "%s", fields[9]);

	gps_data.fix_valid = (fields[2][0] == 'A') ? 1U : 0U;
	gps_data.latitude_deg = GPS_ParseLatitude(fields[3], fields[4]);
	gps_data.longitude_deg = GPS_ParseLongitude(fields[5], fields[6]);
	gps_data.speed_knots = (fields[7][0] != '\0') ? (float)atof(fields[7]) : 0.0f;
	gps_data.speed_kph = gps_data.speed_knots * 1.852f;
	gps_data.course_deg = (fields[8][0] != '\0') ? (float)atof(fields[8]) : 0.0f;

	gps_diag.rmc_count++;
}

static void GPS_ParseGGA(const char *sentence)
{
	char buffer[GPS_LINE_BUFFER_SIZE];
	char *fields[20] = {0};
	uint32_t field_count;

	snprintf(buffer, sizeof(buffer), "%s", sentence);
	field_count = GPS_SplitFields(buffer, fields, 20U);

	if (field_count < 10U)
	{
		return;
	}

	snprintf((char *)gps_data.utc_time, sizeof(gps_data.utc_time), "%s", fields[1]);

	gps_data.latitude_deg = GPS_ParseLatitude(fields[2], fields[3]);
	gps_data.longitude_deg = GPS_ParseLongitude(fields[4], fields[5]);
	gps_data.fix_quality = (fields[6][0] != '\0') ? (uint8_t)atoi(fields[6]) : 0U;
	gps_data.satellites = (fields[7][0] != '\0') ? (uint8_t)atoi(fields[7]) : 0U;
	gps_data.hdop = (fields[8][0] != '\0') ? (float)atof(fields[8]) : 0.0f;
	gps_data.altitude_m = (fields[9][0] != '\0') ? (float)atof(fields[9]) : 0.0f;

	gps_diag.gga_count++;
}

static void GPS_ParseVTG(const char *sentence)
{
	char buffer[GPS_LINE_BUFFER_SIZE];
	char *fields[20] = {0};
	uint32_t field_count;

	snprintf(buffer, sizeof(buffer), "%s", sentence);
	field_count = GPS_SplitFields(buffer, fields, 20U);

	if (field_count < 9U)
	{
		return;
	}

	gps_data.course_deg = (fields[1][0] != '\0') ? (float)atof(fields[1]) : 0.0f;
	gps_data.speed_knots = (fields[5][0] != '\0') ? (float)atof(fields[5]) : 0.0f;
	gps_data.speed_kph = (fields[7][0] != '\0') ? (float)atof(fields[7]) : 0.0f;

	gps_diag.vtg_count++;
}

static void GPS_ParsePQTMTAR(const char *sentence)
{
	char buffer[GPS_LINE_BUFFER_SIZE];
	char *fields[16] = {0};
	uint32_t field_count;

	snprintf(buffer, sizeof(buffer), "%s", sentence);
	field_count = GPS_SplitFields(buffer, fields, 16U);

	if (field_count < 13U)
	{
		return;
	}

	snprintf((char *)gps_data.utc_time, sizeof(gps_data.utc_time), "%s", fields[2]);

	gps_data.heading_quality = (fields[3][0] != '\0') ? (uint8_t)atoi(fields[3]) : 0U;
	gps_data.baseline_length_m = (fields[5][0] != '\0') ? (float)atof(fields[5]) : 0.0f;
	gps_data.pitch_deg = (fields[6][0] != '\0') ? (float)atof(fields[6]) : 0.0f;
	gps_data.heading_deg = (fields[8][0] != '\0') ? (float)atof(fields[8]) : 0.0f;
	gps_data.heading_accuracy_deg = (fields[11][0] != '\0') ? (float)atof(fields[11]) : 0.0f;
	gps_data.heading_valid = (uint8_t)((((gps_data.heading_quality == 4U)
			|| (gps_data.heading_quality == 5U))
			&& (fields[8][0] != '\0')) ? 1U : 0U);

	gps_diag.pqtmtar_count++;

	if (gps_data.heading_valid != 0U)
	{
		gps_diag.heading_message_enabled = 1U;
	}
}

static void GPS_ParsePAIR001(const char *sentence)
{
	char buffer[GPS_LINE_BUFFER_SIZE];
	char *fields[8] = {0};
	uint32_t field_count;

	snprintf(buffer, sizeof(buffer), "%s", sentence);
	field_count = GPS_SplitFields(buffer, fields, 8U);

	if (field_count < 3U)
	{
		return;
	}

	gps_diag.pair001_count++;
	gps_diag.pair001_last_result = (int8_t)atoi(fields[2]);
	snprintf((char *)gps_diag.pair001_last_msgid, sizeof(gps_diag.pair001_last_msgid), "%s", fields[1]);
}

static void GPS_ParsePQTMCFGMSGRATE(const char *sentence)
{
	char buffer[GPS_LINE_BUFFER_SIZE];
	char *fields[8] = {0};
	uint32_t field_count;
	uint32_t rate;
	uint32_t version;

	snprintf(buffer, sizeof(buffer), "%s", sentence);
	field_count = GPS_SplitFields(buffer, fields, 8U);

	if (field_count < 2U)
	{
		return;
	}

	if (strcmp(fields[1], "OK") == 0)
	{
		if (field_count == 2U)
		{
			gps_diag.config_command_status = (uint8_t)HAL_OK;

			if (gps_config_stage == GPS_CONFIG_STAGE_WAIT_ENABLE_ACK)
			{
				gps_config_stage = GPS_CONFIG_STAGE_QUERY_READBACK;
				GPS_AllowImmediateRetry();
			}

			return;
		}

		if ((field_count < 5U) || (strcmp(fields[2], "PQTMTAR") != 0))
		{
			return;
		}

		gps_diag.config_readback_count++;
		rate = (fields[3][0] != '\0') ? (uint32_t)strtoul(fields[3], NULL, 10) : 0U;
		version = (fields[4][0] != '\0') ? (uint32_t)strtoul(fields[4], NULL, 10) : 0U;

		if ((rate == 1U) && (version == 1U))
		{
			gps_diag.config_readback_status = (uint8_t)HAL_OK;
			gps_diag.heading_message_enabled = 1U;
			gps_diag.heading_message_verified = 1U;
			gps_config_stage = GPS_CONFIG_STAGE_VERIFIED;
			gps_config_retry_exhausted = 0U;
		}
		else
		{
			gps_diag.config_readback_status = (uint8_t)HAL_ERROR;
			gps_diag.heading_message_enabled = 0U;
			gps_diag.heading_message_verified = 0U;
			gps_config_stage = GPS_CONFIG_STAGE_ENABLE_WRITE;
			GPS_AllowImmediateRetry();
		}

		return;
	}

	if (strcmp(fields[1], "ERROR") == 0)
	{
		if ((field_count >= 3U) && (strcmp(fields[2], "PQTMTAR") == 0))
		{
			gps_diag.config_readback_status = (uint8_t)HAL_ERROR;
		}
		else
		{
			gps_diag.config_command_status = (uint8_t)HAL_ERROR;
		}

		gps_diag.heading_message_enabled = 0U;
		gps_diag.heading_message_verified = 0U;
		gps_config_stage = GPS_CONFIG_STAGE_ENABLE_WRITE;
		GPS_AllowImmediateRetry();
	}
}

static void GPS_ParsePQTMTAR(const char *sentence)
{
  char buffer[GPS_LINE_BUFFER_SIZE];
  char *fields[16] = {0};
  uint32_t field_count;

  snprintf(buffer, sizeof(buffer), "%s", sentence);
  field_count = GPS_SplitFields(buffer, fields, 16U);

  if (field_count < 13U)
  {
    return;
  }

  snprintf((char *)gps_data.utc_time, sizeof(gps_data.utc_time), "%s", fields[2]);

  gps_data.heading_quality = (fields[3][0] != '\0') ? (uint8_t)atoi(fields[3]) : 0U;
  gps_data.baseline_length_m = (fields[5][0] != '\0') ? (float)atof(fields[5]) : 0.0f;
  gps_data.pitch_deg = (fields[6][0] != '\0') ? (float)atof(fields[6]) : 0.0f;
  gps_data.heading_deg = (fields[8][0] != '\0') ? (float)atof(fields[8]) : 0.0f;
  gps_data.heading_accuracy_deg = (fields[11][0] != '\0') ? (float)atof(fields[11]) : 0.0f;
  gps_data.heading_valid = (uint8_t)((((gps_data.heading_quality == 4U)
                                       || (gps_data.heading_quality == 5U))
                                      && (fields[8][0] != '\0')) ? 1U : 0U);

  gps_pqtmtar_count++;
}

static void GPS_HandleSentence(const char *sentence)
{
	if (sentence == NULL)
	{
		return;
	}

	if ((strncmp(sentence, "$GNRMC", 6) == 0) || (strncmp(sentence, "$GPRMC", 6) == 0))
	{
		GPS_ParseRMC(sentence);
	}
	else if ((strncmp(sentence, "$GNGGA", 6) == 0) || (strncmp(sentence, "$GPGGA", 6) == 0))
	{
		GPS_ParseGGA(sentence);
	}
	else if ((strncmp(sentence, "$GNVTG", 6) == 0) || (strncmp(sentence, "$GPVTG", 6) == 0))
	{
		GPS_ParseVTG(sentence);
	}
	else if (strncmp(sentence, "$PQTMTAR", 8) == 0)
	{
		GPS_ParsePQTMTAR(sentence);
	}
	else if (strncmp(sentence, "$PQTMCFGMSGRATE", 15) == 0)
	{
		GPS_ParsePQTMCFGMSGRATE(sentence);
	}
	else if (strncmp(sentence, "$PAIR001", 8) == 0)
	{
		GPS_ParsePAIR001(sentence);
	}
}

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart)
{
	if (uart == NULL)
	{
		return HAL_ERROR;
	}

	gps_uart = uart;
	gps_config_stage = GPS_CONFIG_STAGE_ENABLE_WRITE;
	gps_last_command_ms = 0U;
	gps_config_retry_count = 0U;
	gps_config_retry_exhausted = 0U;

	memset((void *)&gps_data, 0, sizeof(gps_data));
	memset((void *)&gps_diag, 0, sizeof(gps_diag));
	memset((void *)LIVE_CMD, 0, sizeof(LIVE_CMD));
	memset((void *)LIVE_CMD_TEXT, 0, sizeof(LIVE_CMD_TEXT));
	memset(gps_last_live_cmd, 0, sizeof(gps_last_live_cmd));
	memset(gps_last_live_cmd_text, 0, sizeof(gps_last_live_cmd_text));
	gps_diag.pair001_last_result = -1;
	GPS_ResetRxState();

	gps_diag.init_count = 1U;
	gps_diag.active_baud_rate = gps_uart->Init.BaudRate;
	gps_diag.detected_baud_rate = gps_uart->Init.BaudRate;
	gps_diag.baud_locked = 1U;
	gps_diag.uart_rx_pin_level = GPS_ReadActiveRxPinLevel();
	gps_diag.uart_rx_pin_last_level = gps_diag.uart_rx_pin_level;

	//  if (gps_uart->Init.BaudRate != GPS_DEFAULT_BAUD_RATE)
	//  {
	//    gps_diag.start_receive_status = (uint8_t)HAL_ERROR;
	//    GPS_UpdateUartDiagState();
	//    return HAL_ERROR;
	//  }

	GPS_UpdateUartDiagState();

	/* Arm RX so bytes arriving during the restore reboot are captured */
	if (GPS_StartReceiveIT() != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* Initialise PAIR protocol session */
	(void)GPS_SendCommand(GPS_PAIR_INIT_CMD);
	{
		uint32_t wait_start = HAL_GetTick();
		while ((HAL_GetTick() - wait_start) < 200U)
		{
			GPS_ProcessPendingBytes();
		}
	}

	/* Query firmware version */
	(void)GPS_SendCommand(GPS_QUERY_VER_CMD);
	{
		uint32_t wait_start = HAL_GetTick();
		while ((HAL_GetTick() - wait_start) < 200U)
		{
			GPS_ProcessPendingBytes();
		}
	}

	/* Enable GGA and RMC output; module ACKs with $PQTMCFGMSGRATE,OK */
	(void)GPS_SendCommand(GPS_ENABLE_GGA_CMD);
	(void)GPS_SendCommand(GPS_ENABLE_RMC_CMD);
	{
		uint32_t wait_start = HAL_GetTick();
		while ((HAL_GetTick() - wait_start) < 1000U)
		{
			GPS_ProcessPendingBytes();
		}
	}

	if (GPS_SendCommand(GPS_PQTMTAR_ENABLE_CMD) == HAL_OK)
	{
		gps_config_stage = GPS_CONFIG_STAGE_WAIT_ENABLE_ACK;
		gps_last_command_ms = HAL_GetTick();
	}
	{
		uint32_t wait_start = HAL_GetTick();
		while ((HAL_GetTick() - wait_start) < 1000U)
		{
			GPS_ProcessPendingBytes();
		}
	}

	if (GPS_SendCommand(GPS_PQTMTAR_QUERY_CMD) == HAL_OK)
	{
		gps_diag.config_readback_status = (uint8_t)HAL_BUSY;
		gps_config_stage = GPS_CONFIG_STAGE_WAIT_READBACK;
		gps_last_command_ms = HAL_GetTick();
	}
	{
		uint32_t wait_start = HAL_GetTick();
		while ((HAL_GetTick() - wait_start) < 1000U)
		{
			GPS_ProcessPendingBytes();
		}
	}




//	HAL_UART_Transmit(gps_uart, (uint8_t *)en_gga_cmd,
//			(uint16_t)(sizeof(en_gga_cmd) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(gps_uart, (uint8_t *)en_rmc_cmd,
//			(uint16_t)(sizeof(en_rmc_cmd) - 1U), 100U);
//	HAL_Delay(1000U);
//	HAL_UART_Transmit(gps_uart, (uint8_t *)en_vtg_cmd,
//			(uint16_t)(sizeof(en_vtg_cmd) - 1U), 100U);
//	HAL_Delay(1000U);
	GPS_SendCommand(GPS_EN_VTG_PAIR_CMD);
	{
			uint32_t wait_start = HAL_GetTick();
			while ((HAL_GetTick() - wait_start) < 2000U)
			{
				GPS_ProcessPendingBytes();
			}
		}
	GPS_SendCommand(GPS_EN_GGA_PAIR_CMD);
	{
			uint32_t wait_start = HAL_GetTick();
			while ((HAL_GetTick() - wait_start) < 2000U)
			{
				GPS_ProcessPendingBytes();
			}
		}
	GPS_SendCommand(GPS_EN_RMC_PAIR_CMD);
	{
			uint32_t wait_start = HAL_GetTick();
			while ((HAL_GetTick() - wait_start) < 2000U)
			{
				GPS_ProcessPendingBytes();
			}
		}

	return HAL_OK;
}

HAL_StatusTypeDef GPS_StartReceiveIT(void)
{
	HAL_StatusTypeDef status;

	gps_diag.start_receive_calls++;

	if (gps_uart == NULL)
	{
		gps_diag.start_receive_status = (uint8_t)HAL_ERROR;
		GPS_UpdateUartDiagState();
		return HAL_ERROR;
	}

	status = GPS_StartReceiveDma();

	return status;
}

void GPS_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if ((gps_uart == NULL) || (huart == NULL) || (huart->Instance != gps_uart->Instance))
	{
		return;
	}

	GPS_DrainRxDma();
}

void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((gps_uart == NULL) || (huart == NULL) || (huart->Instance != gps_uart->Instance))
	{
		return;
	}

	GPS_DrainRxDma();
}

void GPS_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == NULL)
	{
		return;
	}

	if ((gps_uart != NULL) && (huart->Instance == gps_uart->Instance))
	{
		gps_tx_busy = 0U;
	}

	if (huart->Instance == huart2.Instance)
	{
		gps_uart2_tx_busy = 0U;
		GPS_KickUart2Tx();
	}
}

void GPS_Process(void)
{
	uint32_t now;

	GPS_SampleRxPinLevel();

	now = HAL_GetTick();

	if ((gps_uart != NULL)
			&& (gps_diag.pqtmtar_count == 0U)
			&& (gps_config_retry_exhausted == 0U)
			&& (gps_config_stage != GPS_CONFIG_STAGE_VERIFIED)
			&& ((now - gps_last_command_ms) >= GPS_CONFIG_RETRY_MS))
	{
		if (gps_config_retry_count >= GPS_CONFIG_MAX_RETRIES)
		{
			gps_config_retry_exhausted = 1U;
			gps_diag.config_readback_status = (uint8_t)HAL_TIMEOUT;
		}
		else if ((gps_config_stage == GPS_CONFIG_STAGE_ENABLE_WRITE)
				|| (gps_config_stage == GPS_CONFIG_STAGE_WAIT_ENABLE_ACK))
		{
			if (GPS_SendCommand(GPS_PQTMTAR_ENABLE_CMD) == HAL_OK)
			{
				gps_config_stage = GPS_CONFIG_STAGE_WAIT_ENABLE_ACK;
				gps_last_command_ms = now;
				gps_config_retry_count++;
			}
		}
		else
		{
			if (GPS_SendCommand(GPS_PQTMTAR_QUERY_CMD) == HAL_OK)
			{
				gps_diag.config_readback_status = (uint8_t)HAL_BUSY;
				gps_config_stage = GPS_CONFIG_STAGE_WAIT_READBACK;
				gps_last_command_ms = now;
				gps_config_retry_count++;
			}
		}
	}

	GPS_ProcessPendingBytes();
	GPS_KickUart2Tx();
	GPS_ProcessLiveCommandText();
	GPS_ProcessLiveCommand();
}

static void GPS_ProcessLiveCommandText(void)
{
	char live_cmd_text_snapshot[GPS_LINE_BUFFER_SIZE];
	size_t live_cmd_text_len;
	size_t compare_len;

	memcpy(live_cmd_text_snapshot, (const void *)LIVE_CMD_TEXT, sizeof(live_cmd_text_snapshot));
	live_cmd_text_snapshot[GPS_LINE_BUFFER_SIZE - 1U] = '\0';

	live_cmd_text_len = GPS_BoundedStringLength(live_cmd_text_snapshot, GPS_LINE_BUFFER_SIZE - 1U);
	compare_len = GPS_BoundedStringLength(gps_last_live_cmd_text, GPS_LINE_BUFFER_SIZE - 1U);

	if ((live_cmd_text_len == compare_len)
			&& (memcmp(live_cmd_text_snapshot, gps_last_live_cmd_text, live_cmd_text_len) == 0))
	{
		return;
	}

	memcpy(gps_last_live_cmd_text, live_cmd_text_snapshot, sizeof(gps_last_live_cmd_text));
	GPS_BuildCommandFromText(live_cmd_text_snapshot, (char *)LIVE_CMD, GPS_LINE_BUFFER_SIZE);
}

static void GPS_ProcessLiveCommand(void)
{
	char live_cmd_snapshot[GPS_LINE_BUFFER_SIZE];
	size_t live_cmd_len;
	size_t compare_len;

	if (gps_uart == NULL)
	{
		return;
	}

	memcpy(live_cmd_snapshot, (const void *)LIVE_CMD, sizeof(live_cmd_snapshot));
	live_cmd_snapshot[GPS_LINE_BUFFER_SIZE - 1U] = '\0';

	live_cmd_len = GPS_BoundedStringLength(live_cmd_snapshot, GPS_LINE_BUFFER_SIZE - 1U);
	compare_len = GPS_BoundedStringLength(gps_last_live_cmd, GPS_LINE_BUFFER_SIZE - 1U);

	if ((live_cmd_len == compare_len) && (memcmp(live_cmd_snapshot, gps_last_live_cmd, live_cmd_len) == 0))
	{
		return;
	}

	memcpy(gps_last_live_cmd, live_cmd_snapshot, sizeof(gps_last_live_cmd));

	if (live_cmd_len == 0U)
	{
		return;
	}

	(void)GPS_SendCommand(live_cmd_snapshot);
}

static void GPS_ProcessPendingBytes(void)
{
	GPS_DrainRxDma();

	while (gps_rx_tail != gps_rx_head)
	{
		char c = (char)gps_rx_buffer[gps_rx_tail];
		gps_rx_tail = GPS_NextIndex(gps_rx_tail);

		if ((c == '\r') || (c == '\n'))
		{
			if (gps_line_index > 0U)
			{
				gps_line_buffer[gps_line_index] = '\0';

				snprintf((char *)gps_diag.last_sentence,
						GPS_LINE_BUFFER_SIZE,
						"%s",
						gps_line_buffer);

				gps_diag.sentence_ready = 1U;
				gps_diag.sentence_count++;
				gps_line_index = 0U;

				GPS_HandleSentence((const char *)gps_diag.last_sentence);
				GPS_MirrorSentenceToUart2((const char *)gps_diag.last_sentence);
				GPS_MirrorDataCsvToUart2();
				gps_diag.sentence_ready = 0U;
			}
		}
		else
		{
			if (gps_line_index < (GPS_LINE_BUFFER_SIZE - 1U))
			{
				gps_line_buffer[gps_line_index++] = c;
			}
			else
			{
				gps_line_index = 0U;
			}
		}
	}
}
