#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define GPS_DMA_BUF_SIZE    1024U
#define GPS_LINE_BUF_SIZE   128U
#define GPS_TX_BUF_SIZE     128U
#define GPS_DBG_QUEUE_SIZE  2048U
#define GPS_DBG_TX_BUF_SIZE 256U

/* Enable NMEA message types via PAIR062, save via PAIR513 */
static const char k_en_gga[] = "$PAIR062,0,1*3F\r\n";
static const char k_en_rmc[] = "$PAIR062,4,1*3B\r\n";
static const char k_en_vtg[] = "$PAIR062,5,1*3A\r\n";
static const char k_save[]   = "$PAIR513*3D\r\n";

static UART_HandleTypeDef *s_uart = NULL;

static uint8_t   s_dma_buf[GPS_DMA_BUF_SIZE];
static uint16_t  s_dma_last = 0U;

static char      s_line[GPS_LINE_BUF_SIZE];
static uint16_t  s_line_len = 0U;

static uint8_t          s_dbg_queue[GPS_DBG_QUEUE_SIZE];
static volatile uint16_t s_dbg_head = 0U;
static volatile uint16_t s_dbg_tail = 0U;
static uint8_t          s_dbg_tx_buf[GPS_DBG_TX_BUF_SIZE];
static volatile uint8_t  s_dbg_busy = 0U;
static volatile uint8_t  s_drain_busy = 0U;

volatile GPS_Data_t gps_data = {0};
volatile GPS_Diag_t gps_diag = {0};

extern UART_HandleTypeDef huart2;

/* ── helpers ─────────────────────────────────────────────────────────────── */

static uint32_t split_fields(char *buf, char **fields, uint32_t max)
{
    uint32_t n = 0U;
    char *p = buf;

    if ((buf == NULL) || (fields == NULL) || (max == 0U)) {
        return 0U;
    }

    fields[n++] = p;
    while ((*p != '\0') && (n < max)) {
        if ((*p == ',') || (*p == '*')) {
            *p = '\0';
            fields[n++] = p + 1U;
        }
        p++;
    }
    return n;
}

static float parse_lat(const char *val, const char *ns)
{
    float raw, deg, min, result;
    if ((val == NULL) || (ns == NULL) || (val[0] == '\0')) {
        return 0.0f;
    }
    raw = (float)atof(val);
    deg = (float)((int)(raw / 100.0f));
    min = raw - (deg * 100.0f);
    result = deg + (min / 60.0f);
    if (ns[0] == 'S') { result = -result; }
    return result;
}

static float parse_lon(const char *val, const char *ew)
{
    float raw, deg, min, result;
    if ((val == NULL) || (ew == NULL) || (val[0] == '\0')) {
        return 0.0f;
    }
    raw = (float)atof(val);
    deg = (float)((int)(raw / 100.0f));
    min = raw - (deg * 100.0f);
    result = deg + (min / 60.0f);
    if (ew[0] == 'W') { result = -result; }
    return result;
}

/* ── debug mirror to USART2 ──────────────────────────────────────────────── */

static void dbg_kick(void)
{
    uint16_t head, tail, tx_len;
    uint32_t primask;

    if (huart2.hdmatx == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if ((s_dbg_busy != 0U) || (s_dbg_head == s_dbg_tail)) {
        if (primask == 0U) { __enable_irq(); }
        return;
    }

    tail = s_dbg_tail;
    head = s_dbg_head;
    tx_len = 0U;

    while ((tail != head) && (tx_len < GPS_DBG_TX_BUF_SIZE)) {
        s_dbg_tx_buf[tx_len++] = s_dbg_queue[tail];
        tail = (uint16_t)((tail + 1U) % GPS_DBG_QUEUE_SIZE);
    }

    s_dbg_tail = tail;
    s_dbg_busy = 1U;

    if (primask == 0U) { __enable_irq(); }

    if (HAL_UART_Transmit_DMA(&huart2, s_dbg_tx_buf, tx_len) != HAL_OK) {
        primask = __get_PRIMASK();
        __disable_irq();
        s_dbg_busy = 0U;
        if (primask == 0U) { __enable_irq(); }
    }
}

static void dbg_push(const uint8_t *data, uint16_t len)
{
    uint16_t i, next;
    uint32_t primask;

    if ((data == NULL) || (len == 0U) || (huart2.hdmatx == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0U; i < len; i++) {
        next = (uint16_t)((s_dbg_head + 1U) % GPS_DBG_QUEUE_SIZE);
        if (next == s_dbg_tail) { break; } /* drop if full */
        s_dbg_queue[s_dbg_head] = data[i];
        s_dbg_head = next;
    }

    if (primask == 0U) { __enable_irq(); }

    dbg_kick();
}

void GPS_DebugMirror(const uint8_t *data, uint16_t len)
{
    dbg_push(data, len);
}

/* ── NMEA parsers ─────────────────────────────────────────────────────────── */

static void parse_rmc(char *buf)
{
    char *f[20] = {0};
    if (split_fields(buf, f, 20U) < 10U) { return; }

    gps_data.fix_valid   = (f[2][0] == 'A') ? 1U : 0U;
    gps_data.latitude_deg  = parse_lat(f[3], f[4]);
    gps_data.longitude_deg = parse_lon(f[5], f[6]);
    gps_data.speed_kph   = (f[7][0] != '\0') ? ((float)atof(f[7]) * 1.852f) : 0.0f;
    gps_data.course_deg  = (f[8][0] != '\0') ? (float)atof(f[8]) : 0.0f;

    gps_diag.rmc_count++;
}

static void parse_gga(char *buf)
{
    char *f[20] = {0};
    if (split_fields(buf, f, 20U) < 10U) { return; }

    gps_data.latitude_deg  = parse_lat(f[2], f[3]);
    gps_data.longitude_deg = parse_lon(f[4], f[5]);
    gps_data.fix_quality = (f[6][0] != '\0') ? (uint8_t)atoi(f[6]) : 0U;
    gps_data.satellites  = (f[7][0] != '\0') ? (uint8_t)atoi(f[7]) : 0U;
    gps_data.hdop        = (f[8][0] != '\0') ? (float)atof(f[8]) : 0.0f;
    gps_data.altitude_m  = (f[9][0] != '\0') ? (float)atof(f[9]) : 0.0f;

    gps_diag.gga_count++;
}

static void parse_vtg(char *buf)
{
    char *f[20] = {0};
    if (split_fields(buf, f, 20U) < 8U) { return; }

    gps_data.course_deg = (f[1][0] != '\0') ? (float)atof(f[1]) : 0.0f;
    gps_data.speed_kph  = (f[7][0] != '\0') ? (float)atof(f[7]) : 0.0f;

    gps_diag.vtg_count++;
}

/* ── sentence handler ─────────────────────────────────────────────────────── */

static void handle_sentence(const char *s)
{
    char buf[GPS_LINE_BUF_SIZE];
    char dbg[GPS_LINE_BUF_SIZE + 8U];
    int  n;

    gps_diag.sentences++;
    gps_diag.sentence_count++;

    n = snprintf(dbg, sizeof(dbg), "GPS>%s\r\n", s);
    if (n > 0) {
        dbg_push((const uint8_t *)dbg, (uint16_t)n);
    }

    if (s[0] == '$' && strlen(s) >= 6) {
        if (strncmp(s + 3, "RMC", 3) == 0) {
            snprintf(buf, sizeof(buf), "%s", s);
            parse_rmc(buf);
        } else if (strncmp(s + 3, "GGA", 3) == 0) {
            snprintf(buf, sizeof(buf), "%s", s);
            parse_gga(buf);
        } else if (strncmp(s + 3, "VTG", 3) == 0) {
            snprintf(buf, sizeof(buf), "%s", s);
            parse_vtg(buf);
        }
    }
}

/* ── byte feed ────────────────────────────────────────────────────────────── */

static void feed_byte(uint8_t b)
{
    gps_diag.last_byte = b;
    gps_diag.rx_bytes++;

    if (b == '$') {
        s_line_len = 0U;
        s_line[s_line_len++] = (char)b;
    } else if ((b == '\r') || (b == '\n')) {
        if ((s_line_len > 0U) && (s_line[0] == '$')) {
            s_line[s_line_len] = '\0';
            handle_sentence(s_line);
        }
        s_line_len = 0U;
    } else if ((b >= 0x20U) && (b <= 0x7EU) && (s_line_len > 0U)) {
        if (s_line_len < (GPS_LINE_BUF_SIZE - 1U)) {
            s_line[s_line_len++] = (char)b;
        } else {
            s_line_len = 0U; /* line too long, discard */
        }
    } else {
        s_line_len = 0U; /* non-printable mid-line = RTCM or corruption */
    }
}

/* ── DMA drain ────────────────────────────────────────────────────────────── */

static void drain_dma(void)
{
    uint16_t pos;
    uint32_t primask;

    if ((s_uart == NULL) || (s_uart->hdmarx == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_drain_busy != 0U) {
        if (primask == 0U) { __enable_irq(); }
        return;
    }
    s_drain_busy = 1U;
    if (primask == 0U) { __enable_irq(); }

    pos = (uint16_t)(GPS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(s_uart->hdmarx));
    if (pos >= GPS_DMA_BUF_SIZE) { pos = 0U; }

    if (pos == s_dma_last) {
        s_drain_busy = 0U;
        return;
    }

    if (pos > s_dma_last) {
        for (uint16_t i = s_dma_last; i < pos; i++) {
            feed_byte(s_dma_buf[i]);
        }
    } else {
        for (uint16_t i = s_dma_last; i < GPS_DMA_BUF_SIZE; i++) {
            feed_byte(s_dma_buf[i]);
        }
        for (uint16_t i = 0U; i < pos; i++) {
            feed_byte(s_dma_buf[i]);
        }
    }

    s_dma_last = pos;
    s_drain_busy = 0U;
}

/* ── DMA RX start ─────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef start_rx_dma(void)
{
    HAL_StatusTypeDef status;

    if (s_uart == NULL) { return HAL_ERROR; }

    if ((s_uart->RxState == HAL_UART_STATE_BUSY_RX) ||
        (s_uart->RxState == HAL_UART_STATE_BUSY_TX_RX)) {
        (void)HAL_UART_AbortReceive(s_uart);
    }

    __HAL_UART_CLEAR_FLAG(s_uart, UART_CLEAR_OREF | UART_CLEAR_FEF
                          | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_IDLEF);
    s_uart->ErrorCode = HAL_UART_ERROR_NONE;

    s_dma_last = 0U;
    status = HAL_UART_Receive_DMA(s_uart, s_dma_buf, GPS_DMA_BUF_SIZE);
    gps_diag.start_status = (uint8_t)status;
    return status;
}

/* ── public API ───────────────────────────────────────────────────────────── */

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart)
{
    if (uart == NULL) { return HAL_ERROR; }

    s_uart = uart;
    s_dma_last = 0U;
    s_line_len = 0U;
    s_dbg_head = 0U;
    s_dbg_tail = 0U;
    s_dbg_busy = 0U;
    s_drain_busy = 0U;

    memset((void *)&gps_data, 0, sizeof(gps_data));
    memset((void *)&gps_diag, 0, sizeof(gps_diag));

    /* Release GPS from reset, wait for boot */
    HAL_GPIO_WritePin(GPS1_RST_GPIO_Port, GPS1_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(500U);

    /* Arm circular DMA RX before sending commands */
    if (start_rx_dma() != HAL_OK) {
        return HAL_ERROR;
    }

    /* Send NMEA enables — blocking TX, short strings, fast at 921600 */
    HAL_UART_Transmit(s_uart, (uint8_t *)k_en_gga, (uint16_t)(sizeof(k_en_gga) - 1U), 50U);
    HAL_UART_Transmit(s_uart, (uint8_t *)k_en_rmc, (uint16_t)(sizeof(k_en_rmc) - 1U), 50U);
    HAL_UART_Transmit(s_uart, (uint8_t *)k_en_vtg, (uint16_t)(sizeof(k_en_vtg) - 1U), 50U);
    HAL_UART_Transmit(s_uart, (uint8_t *)k_save,   (uint16_t)(sizeof(k_save)   - 1U), 50U);

    return HAL_OK;
}

void GPS_Process(void)
{
    drain_dma();
    dbg_kick();
}

void GPS_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
    /* drain happens in main loop via GPS_Process() to avoid s_line race */
}

void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
    /* drain happens in main loop via GPS_Process() to avoid s_line race */
}

void GPS_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == huart2.Instance)) {
        s_dbg_busy = 0U;
        dbg_kick();
    }
}

void GPS_ErrorRestart(void)
{
    (void)start_rx_dma();
}
