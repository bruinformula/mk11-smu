#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GPS_DEFAULT_BAUD_RATE       460800U
#define GPS_PQTMTAR_ENABLE_CMD      "$PQTMCFGMSGRATE,W,PQTMTAR,1,1*09\r\n"
#define GPS_PQTMTAR_QUERY_CMD       "$PQTMCFGMSGRATE,R,PQTMTAR,1*11\r\n"
/* Per LC29H Protocol Spec the 4-arg form is:
 *   $PQTMCFGMSGRATE,W,<MsgName>,<Rate>,<MsgVer>*CS
 * RATE comes before MsgVer. Our earlier "W,RMC,0,1" was parsed by the module
 * as Rate=0 (DISABLE), MsgVer=1 — i.e. we were turning standard NMEA OFF every
 * boot. Corrected to Rate=1, MsgVer=0. Checksums are identical to the swapped
/* PQTMCFGMSGRATE format: W,<MsgName>,<MsgVer>,<Rate>. Standard NMEA uses
 * MsgVer=0; Rate=1 means 1 Hz output. (PQTMTAR uses MsgVer=1.) Note the
 * checksum is symmetric across the trailing two-digit field swap, so a
 * ,1,0 typo produces a syntactically valid "disable" command — easy to
 * miss without verifying argument order. */
#define GPS_EN_RMC_CMD              "$PQTMCFGMSGRATE,W,RMC,0,1*0B\r\n"
#define GPS_EN_VTG_CMD              "$PQTMCFGMSGRATE,W,VTG,0,1*12\r\n"
#define GPS_EN_GGA_CMD              "$PQTMCFGMSGRATE,W,GGA,0,1*16\r\n"
#define GPS_EN_GLL_CMD              "$PQTMCFGMSGRATE,W,GLL,0,1*10\r\n"
#define GPS_EN_GSA_CMD              "$PQTMCFGMSGRATE,W,GSA,0,1*02\r\n"
/* PAIR062 fallback for firmware variants (incl. LC29HEANR11A03S_RSA) that
 * silently ignore PQTMCFGMSGRATE for standard NMEA classes. IDs: 0=GGA,
 * 1=GLL, 2=GSA, 4=RMC, 5=VTG. Acked via $PAIR001,062,<result>. */
#define GPS_PAIR062_EN_GGA_CMD      "$PAIR062,0,1*3F\r\n"
#define GPS_PAIR062_EN_GLL_CMD      "$PAIR062,1,1*3E\r\n"
#define GPS_PAIR062_EN_GSA_CMD      "$PAIR062,2,1*3D\r\n"
#define GPS_PAIR062_EN_RMC_CMD      "$PAIR062,4,1*3B\r\n"
#define GPS_PAIR062_EN_VTG_CMD      "$PAIR062,5,1*3A\r\n"
#define GPS_DISABLE_GGA_CMD          "$PQTMCFGMSGRATE,W,GGA,0*08\r\n"
#define GPS_DISABLE_RMC_CMD          "$PQTMCFGMSGRATE,W,RMC,0*16\r\n"
#define GPS_SAVEPAR_CMD             "$PQTMSAVEPAR*5A\r\n"
/* PAIR513 saves PAIR-family settings (incl. PAIR062 NMEA enables) to flash.
 * PQTMSAVEPAR only persists PQTM-family settings, so without this the
 * PAIR062 enables are lost on power-cycle. */
#define GPS_PAIR513_SAVE_CMD        "$PAIR513*3D\r\n"
/* Diagnostic queries — fired once at boot, responses appear in BASE> log
 * stream so the host can confirm the actual on-device configuration. */
#define GPS_QUERY_RCVRMODE_CMD      "$PQTMCFGRCVRMODE,R*32\r\n"
#define GPS_QUERY_RTCM1074_CMD      "$PQTMCFGMSGRATE,R,1074,1*4C\r\n"
#define GPS_QUERY_VERNO_CMD         "$PQTMVERNO*58\r\n"
#define GPS_COMMAND_RETRY_MS        500U
#define GPS_INIT_TAR_ACK_RETRY_LIMIT 4U
#define GPS_BASE_RX_GPIO_PORT        GPIOB
#define GPS_BASE_RX_PIN              GPIO_PIN_11
#define GPS_DEBUG                    1U
#define GPS_DMA_RX_BUFFER_SIZE       2048U
#define GPS_UART3_TX_BUFFER_SIZE     128U
#define GPS_UART2_TX_BUFFER_SIZE     256U
#define GPS_UART2_TX_QUEUE_SIZE      2048U
#define GPS_TRANSPORT_PREFLIGHT_UART3_PRESENT  (1UL << 0)
#define GPS_TRANSPORT_PREFLIGHT_RX_DMA_LINKED  (1UL << 1)
#define GPS_TRANSPORT_PREFLIGHT_TX_DMA_LINKED  (1UL << 2)
#define GPS_TRANSPORT_PREFLIGHT_USART3_IRQ     (1UL << 3)
#define GPS_TRANSPORT_PREFLIGHT_DMA_RX_IRQ     (1UL << 4)
#define GPS_TRANSPORT_PREFLIGHT_DMA_TX_IRQ     (1UL << 5)
#define GPS_TRANSPORT_PREFLIGHT_ALL_MASK       (GPS_TRANSPORT_PREFLIGHT_UART3_PRESENT \
		| GPS_TRANSPORT_PREFLIGHT_RX_DMA_LINKED \
		| GPS_TRANSPORT_PREFLIGHT_TX_DMA_LINKED \
		| GPS_TRANSPORT_PREFLIGHT_USART3_IRQ \
		| GPS_TRANSPORT_PREFLIGHT_DMA_RX_IRQ \
		| GPS_TRANSPORT_PREFLIGHT_DMA_TX_IRQ)

typedef enum
{
	GPS_INIT_STAGE_IDLE = 0,
	GPS_INIT_STAGE_ENABLE_TAR = 1,
	GPS_INIT_STAGE_QUERY_TAR = 2,
	GPS_INIT_STAGE_SAVEPAR = 3,
	GPS_INIT_STAGE_COMPLETE = 4
} GPS_InitStage_t;

static UART_HandleTypeDef *gps_uart = NULL;
static volatile uint16_t gps_rx_head = 0U;
static volatile uint16_t gps_rx_tail = 0U;
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static uint8_t gps_dma_rx_buffer[GPS_DMA_RX_BUFFER_SIZE];
static volatile uint16_t gps_dma_rx_last_pos = 0U;
static uint8_t gps_uart3_tx_buffer[GPS_UART3_TX_BUFFER_SIZE];
static uint8_t gps_uart2_tx_buffer[GPS_UART2_TX_BUFFER_SIZE];
static uint8_t gps_uart2_tx_queue[GPS_UART2_TX_QUEUE_SIZE];
static volatile uint8_t gps_uart3_tx_busy = 0U;
static volatile uint8_t gps_uart2_tx_busy = 0U;
static volatile uint16_t gps_uart2_tx_head = 0U;
static volatile uint16_t gps_uart2_tx_tail = 0U;

static char gps_line_buffer[GPS_LINE_BUFFER_SIZE];
static uint16_t gps_line_index = 0U;
static GPS_InitStage_t gps_init_stage = GPS_INIT_STAGE_IDLE;
static uint8_t gps_init_command_inflight = 0U;
static uint32_t gps_last_command_ms = 0U;
static uint32_t gps_init_retry_count = 0U;
volatile char LIVE_CMD[GPS_LINE_BUFFER_SIZE] = {0};
volatile char LIVE_CMD_TEXT[GPS_LINE_BUFFER_SIZE] = {0};
static char gps_last_live_cmd[GPS_LINE_BUFFER_SIZE] = {0};
static char gps_last_live_cmd_text[GPS_LINE_BUFFER_SIZE] = {0};

volatile GPS_Data_t gps_data = {0};
volatile GPS_Diag_t gps_diag = {0};
volatile uint32_t gps_init_stage_debug = GPS_INIT_STAGE_IDLE;
volatile uint32_t gps_init_retry_count_debug = 0U;

extern UART_HandleTypeDef huart2;

static void GPS_HandleSentence(const char *sentence);
static void GPS_HandleInitSentence(const char *sentence);
static void GPS_ProcessPendingBytes(void);
static void GPS_ProcessLiveCommandText(void);
static void GPS_ProcessLiveCommand(void);
static void GPS_MirrorCommandToUart2(const char *command);
static void GPS_MirrorSentenceToUart2(const char *sentence);
static void GPS_MirrorDataCsvToUart2(void);
static void GPS_BuildCommandFromText(const char *source, char *dest, size_t dest_size);
static void GPS_PushByte(uint8_t byte);
static HAL_StatusTypeDef GPS_SendCommand(const char *command);
static HAL_StatusTypeDef GPS_StartReceiveDma(void);
static void GPS_DrainRxDma(void);
static void GPS_QueueMirrorBytesToUart2(const uint8_t *data, uint16_t data_len);

void GPS_DebugMirror(const uint8_t *data, uint16_t len)
{
	GPS_QueueMirrorBytesToUart2(data, len);
}
static void GPS_KickUart2Tx(void);
static void GPS_SetInitStage(GPS_InitStage_t stage);
static void GPS_ScheduleInitStageImmediate(GPS_InitStage_t stage, uint32_t now);
static void GPS_CountInitRetry(uint32_t now);
static void GPS_SendStandardNmeaEnableCommands(void);
static uint8_t GPS_RunTransportPreflight(void);
static void GPS_AdvanceInitStage(void);
static void GPS_RequestInitRetry(uint32_t now);
static HAL_StatusTypeDef GPS_SendInitCommandForStage(GPS_InitStage_t stage);
static void GPS_ProcessInitStateMachine(uint32_t now);
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

static void GPS_SetInitStage(GPS_InitStage_t stage)
{
	if (gps_init_stage != stage)
	{
		gps_init_retry_count = 0U;
		gps_init_retry_count_debug = 0U;
	}

	gps_init_stage = stage;
	gps_init_stage_debug = (uint32_t)stage;
}

static void GPS_ScheduleInitStageImmediate(GPS_InitStage_t stage, uint32_t now)
{
	GPS_SetInitStage(stage);
	gps_init_command_inflight = 0U;
	gps_last_command_ms = now - GPS_COMMAND_RETRY_MS;
}

static void GPS_CountInitRetry(uint32_t now)
{
	gps_init_retry_count++;
	gps_init_retry_count_debug = gps_init_retry_count;

	if ((gps_init_stage == GPS_INIT_STAGE_ENABLE_TAR)
			&& (gps_init_retry_count >= GPS_INIT_TAR_ACK_RETRY_LIMIT))
	{
		/* LC29HEANR11A03S_RSA never acks PQTMCFGMSGRATE; falling back to
		 * QUERY_TAR is just as silent. Jump straight to SAVEPAR so PQTMSAVEPAR
		 * gets a chance to commit (and PAIR513 has already fired during init). */
		gps_diag.config_readback_status = (uint8_t)HAL_BUSY;
		GPS_ScheduleInitStageImmediate(GPS_INIT_STAGE_SAVEPAR, now);
	}
	else if ((gps_init_stage == GPS_INIT_STAGE_SAVEPAR)
			&& (gps_init_retry_count >= GPS_INIT_TAR_ACK_RETRY_LIMIT))
	{
		/* Same firmware-silence escape hatch for SAVEPAR \u2014 if PQTMSAVEPAR is
		 * not acked either, give up waiting and mark init complete so the
		 * NMEA parse path runs freely. */
		GPS_ScheduleInitStageImmediate(GPS_INIT_STAGE_COMPLETE, now);
	}
}

static uint8_t GPS_RunTransportPreflight(void)
{
	uint32_t preflight_mask = 0U;

	if ((gps_uart != NULL) && (gps_uart->Instance == USART3))
	{
		preflight_mask |= GPS_TRANSPORT_PREFLIGHT_UART3_PRESENT;
	}

	if ((gps_uart != NULL) && (gps_uart->hdmarx != NULL))
	{
		preflight_mask |= GPS_TRANSPORT_PREFLIGHT_RX_DMA_LINKED;
	}

	if ((gps_uart != NULL) && (gps_uart->hdmatx != NULL))
	{
		preflight_mask |= GPS_TRANSPORT_PREFLIGHT_TX_DMA_LINKED;
	}

	if (NVIC_GetEnableIRQ(USART3_IRQn) != 0U)
	{
		preflight_mask |= GPS_TRANSPORT_PREFLIGHT_USART3_IRQ;
	}

	if (NVIC_GetEnableIRQ(DMA1_Channel1_IRQn) != 0U)
	{
		preflight_mask |= GPS_TRANSPORT_PREFLIGHT_DMA_RX_IRQ;
	}

	if (NVIC_GetEnableIRQ(DMA1_Channel2_IRQn) != 0U)
	{
		preflight_mask |= GPS_TRANSPORT_PREFLIGHT_DMA_TX_IRQ;
	}

	gps_diag.transport_preflight_mask = preflight_mask;
	gps_diag.transport_preflight_ok = (preflight_mask == GPS_TRANSPORT_PREFLIGHT_ALL_MASK) ? 1U : 0U;

	return gps_diag.transport_preflight_ok;
}

static void GPS_SendStandardNmeaEnableCommands(void)
{
	/* Send both protocol families. Some LC29H firmware builds ack only one of
	 * them; whichever the module honors will turn the message on, the other
	 * is a no-op. Fire-and-forget. PAIR513 at the end persists the PAIR062
	 * settings to flash — must live here (not in the SAVEPAR stage) because
	 * LC29HEANR11A03S_RSA does not ack PQTMCFGMSGRATE so the init state
	 * machine never reaches SAVEPAR. */
	(void)GPS_SendCommand(GPS_EN_RMC_CMD);
	(void)GPS_SendCommand(GPS_EN_VTG_CMD);
	(void)GPS_SendCommand(GPS_EN_GGA_CMD);
	(void)GPS_SendCommand(GPS_EN_GLL_CMD);
	(void)GPS_SendCommand(GPS_EN_GSA_CMD);
	(void)GPS_SendCommand(GPS_PAIR062_EN_RMC_CMD);
	(void)GPS_SendCommand(GPS_PAIR062_EN_VTG_CMD);
	(void)GPS_SendCommand(GPS_PAIR062_EN_GGA_CMD);
	(void)GPS_SendCommand(GPS_PAIR062_EN_GLL_CMD);
	(void)GPS_SendCommand(GPS_PAIR062_EN_GSA_CMD);
	(void)GPS_SendCommand(GPS_PAIR513_SAVE_CMD);

	/* Diagnostic queries — responses arrive on the BASE> stream so the host
	 * can see the on-device receiver mode (1=Base, 2=Moving-baseline Rover)
	 * and confirm GPS MSM4 (1074) is actually enabled. */
	(void)GPS_SendCommand(GPS_QUERY_VERNO_CMD);
	(void)GPS_SendCommand(GPS_QUERY_RCVRMODE_CMD);
	(void)GPS_SendCommand(GPS_QUERY_RTCM1074_CMD);
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

static void GPS_SampleRxPinLevel(void)
{
	uint8_t level = (uint8_t)((((GPS_BASE_RX_GPIO_PORT)->IDR & GPS_BASE_RX_PIN) != 0U) ? 1U : 0U);

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
	gps_uart3_tx_busy = 0U;
	gps_uart2_tx_busy = 0U;
	gps_uart2_tx_head = 0U;
	gps_uart2_tx_tail = 0U;
	gps_init_command_inflight = 0U;
	GPS_SetInitStage(GPS_INIT_STAGE_IDLE);
	gps_diag.sentence_ready = 0U;
	memset(gps_dma_rx_buffer, 0, sizeof(gps_dma_rx_buffer));
	memset(gps_uart2_tx_queue, 0, sizeof(gps_uart2_tx_queue));
	memset((void *)gps_diag.last_sentence, 0, sizeof(gps_diag.last_sentence));
}

static void GPS_AdvanceInitStage(void)
{
	switch (gps_init_stage)
	{
	case GPS_INIT_STAGE_ENABLE_TAR:
		/* Skip QUERY_TAR — LC29H's reply format for Read is inconsistent and the
		 * verification isn't needed for function. Go straight to SAVEPAR. */
		GPS_SetInitStage(GPS_INIT_STAGE_SAVEPAR);
		break;
	case GPS_INIT_STAGE_QUERY_TAR:
		GPS_SetInitStage(GPS_INIT_STAGE_SAVEPAR);
		break;
	case GPS_INIT_STAGE_SAVEPAR:
		GPS_SetInitStage(GPS_INIT_STAGE_COMPLETE);
		break;
	case GPS_INIT_STAGE_IDLE:
	case GPS_INIT_STAGE_COMPLETE:
	default:
		break;
	}

	gps_init_command_inflight = 0U;
}

static void GPS_RequestInitRetry(uint32_t now)
{
	GPS_CountInitRetry(now);

	if (gps_init_stage == GPS_INIT_STAGE_QUERY_TAR)
	{
		return;
	}

	gps_init_command_inflight = 1U;
	gps_last_command_ms = now;
}

static HAL_StatusTypeDef GPS_SendInitCommandForStage(GPS_InitStage_t stage)
{
	const char *command = NULL;

	switch (stage)
	{
	case GPS_INIT_STAGE_ENABLE_TAR:
		command = GPS_PQTMTAR_ENABLE_CMD;
		break;
	case GPS_INIT_STAGE_QUERY_TAR:
		command = GPS_PQTMTAR_QUERY_CMD;
		break;
	case GPS_INIT_STAGE_SAVEPAR:
		/* PAIR513 already fired during GPS_SendStandardNmeaEnableCommands;
		 * here we only need to commit PQTM-family settings via PQTMSAVEPAR. */
		command = GPS_SAVEPAR_CMD;
		break;
	case GPS_INIT_STAGE_IDLE:
	case GPS_INIT_STAGE_COMPLETE:
	default:
		return HAL_OK;
	}

	gps_init_command_inflight = 1U;
	gps_last_command_ms = HAL_GetTick();

	return GPS_SendCommand(command);
}

static void GPS_ProcessInitStateMachine(uint32_t now)
{
	if ((gps_uart == NULL)
			|| (gps_init_stage == GPS_INIT_STAGE_IDLE)
			|| (gps_init_stage == GPS_INIT_STAGE_COMPLETE))
	{
		return;
	}

	if ((gps_init_command_inflight != 0U)
			&& ((now - gps_last_command_ms) >= GPS_COMMAND_RETRY_MS))
	{
		GPS_CountInitRetry(now);
	}

	if ((gps_init_command_inflight == 0U)
			|| ((now - gps_last_command_ms) >= GPS_COMMAND_RETRY_MS))
	{
		(void)GPS_SendInitCommandForStage(gps_init_stage);
	}
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

	/* If a previous DMA is already running, abort it before re-arming so the
	 * call below never returns HAL_BUSY and silently no-ops. */
	if ((gps_uart->RxState == HAL_UART_STATE_BUSY_RX)
			|| (gps_uart->RxState == HAL_UART_STATE_BUSY_TX_RX))
	{
		(void)HAL_UART_AbortReceive(gps_uart);
	}

	/* Clear any latched RX errors (ORE/FE/NE/PE). At 921600 the module is
	 * already streaming when the USART is first enabled, so ORE will be set
	 * on the very first byte and the receive path stays dead until we clear
	 * it explicitly. */
	__HAL_UART_CLEAR_FLAG(gps_uart, UART_CLEAR_OREF | UART_CLEAR_FEF
			| UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_IDLEF);
	gps_uart->ErrorCode = HAL_UART_ERROR_NONE;

	gps_dma_rx_last_pos = 0U;

	status = HAL_UART_Receive_DMA(gps_uart, gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);

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
		/* Raw-mirror this contiguous chunk to USART2 before per-byte parsing. */
#ifndef DEBUG_MIRROR_ROVER_ONLY
		GPS_QueueMirrorBytesToUart2(&gps_dma_rx_buffer[gps_dma_rx_last_pos],
				(uint16_t)(dma_pos - gps_dma_rx_last_pos));
#endif
		while (gps_dma_rx_last_pos < dma_pos)
		{
			GPS_PushByte(gps_dma_rx_buffer[gps_dma_rx_last_pos]);
			gps_dma_rx_last_pos++;
		}
	}
	else
	{
		/* Wrap-around: mirror tail of buffer, then head. */
#ifndef DEBUG_MIRROR_ROVER_ONLY
		GPS_QueueMirrorBytesToUart2(&gps_dma_rx_buffer[gps_dma_rx_last_pos],
				(uint16_t)(GPS_DMA_RX_BUFFER_SIZE - gps_dma_rx_last_pos));
#endif
		while (gps_dma_rx_last_pos < GPS_DMA_RX_BUFFER_SIZE)
		{
			GPS_PushByte(gps_dma_rx_buffer[gps_dma_rx_last_pos]);
			gps_dma_rx_last_pos++;
		}

		gps_dma_rx_last_pos = 0U;

		if (dma_pos > 0U)
		{
#ifndef DEBUG_MIRROR_ROVER_ONLY
			GPS_QueueMirrorBytesToUart2(&gps_dma_rx_buffer[0], dma_pos);
#endif
		}
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
			&gps_uart3_tx_busy,
			gps_uart3_tx_buffer,
			GPS_UART3_TX_BUFFER_SIZE,
			(const uint8_t *)command,
			(uint16_t)strlen(command),
			100U);

	gps_diag.config_command_count++;
	gps_diag.config_command_status = (uint8_t)status;
	gps_diag.uart_last_error_code = gps_uart->ErrorCode;
	GPS_UpdateUartDiagState();

	return status;
}

static void GPS_HandleInitSentence(const char *sentence)
{
	uint32_t now;

	if ((sentence == NULL)
			|| (gps_init_stage == GPS_INIT_STAGE_IDLE)
			|| (gps_init_stage == GPS_INIT_STAGE_COMPLETE))
	{
		return;
	}

	now = HAL_GetTick();

	switch (gps_init_stage)
	{
	case GPS_INIT_STAGE_ENABLE_TAR:
		if (strncmp(sentence, "$PQTMCFGMSGRATE,OK", 18) == 0)
		{
			GPS_AdvanceInitStage();
		}
		else if (strncmp(sentence, "$PQTMCFGMSGRATE,ERROR", 21) == 0)
		{
			GPS_RequestInitRetry(now);
		}
		break;

	case GPS_INIT_STAGE_QUERY_TAR:
		/* LC29H replies to the Read with either a combined
		 *   $PQTMCFGMSGRATE,OK,PQTMTAR,1,1*..  (rate echoed)
		 * or the OK ack plus a separate parameter report. Accept either:
		 * any $PQTMCFGMSGRATE,OK is enough proof the query landed. */
		if (strncmp(sentence, "$PQTMCFGMSGRATE,OK,PQTMTAR,1,1", 30) == 0)
		{
			gps_diag.heading_message_verified = 1U;
			GPS_AdvanceInitStage();
		}
		else if (strncmp(sentence, "$PQTMCFGMSGRATE,OK", 18) == 0)
		{
			GPS_AdvanceInitStage();
		}
		else if (strncmp(sentence, "$PQTMCFGMSGRATE,ERROR", 21) == 0)
		{
			gps_diag.config_readback_status = (uint8_t)HAL_ERROR;
			gps_diag.heading_message_enabled = 0U;
			gps_diag.heading_message_verified = 0U;
			GPS_ScheduleInitStageImmediate(GPS_INIT_STAGE_ENABLE_TAR, now);
		}
		break;

	case GPS_INIT_STAGE_SAVEPAR:
		if (strncmp(sentence, "$PQTMSAVEPAR,OK", 15) == 0)
		{
			GPS_AdvanceInitStage();
		}
		else if (strncmp(sentence, "$PQTMSAVEPAR,ERROR", 18) == 0)
		{
			GPS_RequestInitRetry(now);
		}
		break;

	case GPS_INIT_STAGE_IDLE:
	case GPS_INIT_STAGE_COMPLETE:
	default:
		break;
	}
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
	static const uint8_t prefix[] = "BASE > ";
	uint8_t sentence_buf[(sizeof(prefix) - 1U) + GPS_LINE_BUFFER_SIZE + 2U];

	if ((sentence == NULL) || (sentence[0] == '\0'))
	{
		return;
	}

	sentence_len = (uint16_t)strlen(sentence);
	if (sentence_len > GPS_LINE_BUFFER_SIZE)
	{
		sentence_len = GPS_LINE_BUFFER_SIZE;
	}

	memcpy(sentence_buf, prefix, sizeof(prefix) - 1U);
	memcpy(sentence_buf + (sizeof(prefix) - 1U), sentence, sentence_len);
	sentence_buf[(sizeof(prefix) - 1U) + sentence_len] = '\r';
	sentence_buf[(sizeof(prefix) - 1U) + sentence_len + 1U] = '\n';

	GPS_QueueMirrorBytesToUart2(sentence_buf,
			(uint16_t)((sizeof(prefix) - 1U) + sentence_len + 2U));
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
		if ((*p == ',') || (*p == '*'))
		{
			*p = '\0';

			if (count < max_fields)
			{
				fields[count++] = p + 1;
			}
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
		}
		else
		{
			gps_diag.config_readback_status = (uint8_t)HAL_ERROR;
			gps_diag.heading_message_enabled = 0U;
			gps_diag.heading_message_verified = 0U;
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
	}
}

static void GPS_HandleSentence(const char *sentence)
{
	if (sentence == NULL)
	{
		return;
	}

	GPS_HandleInitSentence(sentence);

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
	gps_last_command_ms = 0U;

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
	gps_diag.uart_rx_pin_level = (uint8_t)((((GPS_BASE_RX_GPIO_PORT)->IDR & GPS_BASE_RX_PIN) != 0U) ? 1U : 0U);
	gps_diag.uart_rx_pin_last_level = gps_diag.uart_rx_pin_level;

	//  if (gps_uart->Init.BaudRate != GPS_DEFAULT_BAUD_RATE)
	//  {
	//    gps_diag.start_receive_status = (uint8_t)HAL_ERROR;
	//    GPS_UpdateUartDiagState();
	//    return HAL_ERROR;
	//  }

	GPS_UpdateUartDiagState();

	if (GPS_RunTransportPreflight() == 0U)
	{
		gps_diag.start_receive_status = (uint8_t)HAL_ERROR;
		return HAL_ERROR;
	}

	/* Arm RX so bytes arriving during the restore reboot are captured */
	if (GPS_StartReceiveIT() != HAL_OK)
	{
		return HAL_ERROR;
	}


	gps_diag.config_readback_status = (uint8_t)HAL_BUSY;

	/* Stateless version probe — if the module's command path works AT ALL we
	 * will see "$PQTMVERNO,<version>*XX" in the mirror within a few hundred
	 * ms. If it never appears, our TX is not reaching the module's parser
	 * (wrong pin, wrong baud, or module's UART is output-only). Sent twice
	 * with a small spacing in case the first lands during an RTCM burst. */
	(void)GPS_SendCommand("$PQTMVERNO*58\r\n");
	HAL_Delay(50U);
	(void)GPS_SendCommand("$PQTMVERNO*58\r\n");
	HAL_Delay(50U);

	/* Module's saved config currently has standard NMEA disabled (only PQTMTAR
	 * and PQTMTXT come out on power-up). Fire enables for GGA/RMC/VTG before
	 * entering the state machine; the SAVEPAR stage below will commit them
	 * to flash along with the PQTMTAR rate. */
	GPS_SendStandardNmeaEnableCommands();

	GPS_SetInitStage(GPS_INIT_STAGE_ENABLE_TAR);

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
		gps_uart3_tx_busy = 0U;
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

	GPS_ProcessPendingBytes();
	GPS_KickUart2Tx();
	now = HAL_GetTick();
	GPS_ProcessInitStateMachine(now);
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

		/* Module multiplexes RTCM 3.x binary frames onto the same UART as NMEA.
		 * Any non-printable byte means we're mid-RTCM, not mid-NMEA — drop the
		 * partial line so the next '$' starts cleanly. */
		if (c == '$')
		{
			gps_line_index = 0U;
			gps_line_buffer[gps_line_index++] = c;
		}
		else if ((c == '\r') || (c == '\n'))
		{
			if ((gps_line_index > 0U) && (gps_line_buffer[0] == '$'))
			{
				gps_line_buffer[gps_line_index] = '\0';

				snprintf((char *)gps_diag.last_sentence,
						GPS_LINE_BUFFER_SIZE,
						"%s",
						gps_line_buffer);

				gps_diag.sentence_ready = 1U;
				gps_diag.sentence_count++;

				GPS_MirrorSentenceToUart2((const char *)gps_diag.last_sentence);
				GPS_HandleSentence((const char *)gps_diag.last_sentence);
				GPS_MirrorDataCsvToUart2();
				gps_diag.sentence_ready = 0U;
			}
			gps_line_index = 0U;
		}
		else if ((c >= 0x20) && (c <= 0x7E) && (gps_line_index > 0U))
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
		else
		{
			gps_line_index = 0U;
		}
	}
}
