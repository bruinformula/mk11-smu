#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GPS_DEFAULT_BAUD_RATE       230400U
#define GPS_PQTMTAR_ENABLE_CMD      "$PQTMCFGMSGRATE,W,PQTMTAR,1,1*09\r\n"
#define GPS_PQTMTAR_QUERY_CMD       "$PQTMCFGMSGRATE,R,PQTMTAR,1*11\r\n"
#define GPS_PAIR_INIT_CMD           "$PAIR002*38\r\n"
#define GPS_QUERY_VER_CMD           "$PQTMQVER*08\r\n"
#define GPS_ENABLE_NMEA_MODE_CMD    "$PAIR100,1,1*3B\r\n"
#define GPS_QUERY_NMEA_MODE_CMD     "$PAIR101*3A\r\n"
#define GPS_QUERY_PORT_UART1_CMD    "$PQTMCFGPORT,R,UART1*32\r\n"
#define GPS_QUERY_PORT_UART2_CMD    "$PQTMCFGPORT,R,UART2*31\r\n"
#define GPS_QUERY_PORT_UART3_CMD    "$PQTMCFGPORT,R,UART3*30\r\n"
#define GPS_ENABLE_PORT_UART3_CMD   "$PQTMCFGPORT,W,UART3,230400,NMEA|RTCM3,NMEA|RTCM3*1C\r\n"
#define GPS_ENABLE_GGA_CMD          "$PQTMCFGMSGRATE,W,GGA,1*0A\r\n"
#define GPS_ENABLE_RMC_CMD          "$PQTMCFGMSGRATE,W,RMC,1*17\r\n"
#define GPS_ENABLE_ALL_NMEA_PAIR_CMD "$PAIR062,-1,1*13\r\n"
#define GPS_ENABLE_GGA_PAIR_CMD     "$PAIR062,0,1*3F\r\n"
#define GPS_ENABLE_RMC_PAIR_CMD     "$PAIR062,4,1*3B\r\n"
#define GPS_QUERY_ALL_NMEA_PAIR_CMD "$PAIR063,-1*0F\r\n"
#define GPS_QUERY_GGA_PAIR_CMD      "$PAIR063,0*23\r\n"
#define GPS_QUERY_RMC_PAIR_CMD      "$PAIR063,4*27\r\n"
#define GPS_DISABLE_GGA_CMD          "$PQTMCFGMSGRATE,W,GGA,0*08\r\n"
#define GPS_DISABLE_RMC_CMD          "$PQTMCFGMSGRATE,W,RMC,0*16\r\n"
#define GPS_SAVEPAR_CMD             "$PQTMSAVEPAR*5A\r\n"
#define GPS_PAIR_SAVE_CMD           "$PAIR513*3D\r\n"
#define GPS_QUERY_UART_CMD          "$PQTMCFGUART,R*36\r\n"
#define GPS_COMMAND_RETRY_MS        1000U
#define GPS_BASE_RX_GPIO_PORT        GPIOB
#define GPS_BASE_RX_PIN              GPIO_PIN_11
#define GPS_DEBUG                    1U

typedef enum
{
  GPS_CONFIG_STAGE_ENABLE_WRITE = 0,
  GPS_CONFIG_STAGE_WAIT_ENABLE_ACK,
  GPS_CONFIG_STAGE_QUERY_READBACK,
  GPS_CONFIG_STAGE_WAIT_READBACK,
  GPS_CONFIG_STAGE_VERIFIED
} GPS_ConfigStage_t;

static UART_HandleTypeDef *gps_uart = NULL;
static uint8_t gps_rx_byte = 0U;
static volatile uint16_t gps_rx_head = 0U;
static volatile uint16_t gps_rx_tail = 0U;
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];

static char gps_line_buffer[GPS_LINE_BUFFER_SIZE];
static uint16_t gps_line_index = 0U;
static GPS_ConfigStage_t gps_config_stage = GPS_CONFIG_STAGE_ENABLE_WRITE;
static uint32_t gps_last_command_ms = 0U;
static uint32_t gps_last_nmea_config_ms = 0U;

volatile GPS_Data_t gps_data = {0};
volatile GPS_Diag_t gps_diag = {0};

extern UART_HandleTypeDef huart2;

static void GPS_HandleSentence(const char *sentence);
static void GPS_ProcessPendingBytes(void);
static void GPS_MirrorCommandToUart2(const char *command);
static void GPS_MirrorSentenceToUart2(const char *sentence);
static void GPS_MirrorDataCsvToUart2(void);

static void GPS_AllowImmediateRetry(void)
{
  gps_last_command_ms = HAL_GetTick() - GPS_COMMAND_RETRY_MS;
}

static uint16_t GPS_NextIndex(uint16_t idx)
{
  return (uint16_t)((idx + 1U) % GPS_RX_BUFFER_SIZE);
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
  gps_rx_byte = 0U;
  gps_diag.sentence_ready = 0U;
  memset((void *)gps_diag.last_sentence, 0, sizeof(gps_diag.last_sentence));
}

static HAL_StatusTypeDef GPS_RearmReceiveIT(void)
{
  HAL_StatusTypeDef status;

  if (gps_uart == NULL)
  {
    gps_diag.start_receive_status = (uint8_t)HAL_ERROR;
    GPS_UpdateUartDiagState();
    return HAL_ERROR;
  }

  status = HAL_UART_Receive_IT(gps_uart, &gps_rx_byte, 1U);

  if ((status == HAL_BUSY) && (gps_uart->RxState == HAL_UART_STATE_BUSY_RX))
  {
    status = HAL_OK;
  }

  gps_diag.start_receive_status = (uint8_t)status;
  gps_diag.uart_last_error_code = gps_uart->ErrorCode;
  GPS_UpdateUartDiagState();

  return status;
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

  status = HAL_UART_Transmit(gps_uart,
                             (uint8_t *)command,
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
  (void)HAL_UART_Transmit(&huart2, echo_buf, echo_len, 20U);
#else
  (void)command;
#endif
}

static void GPS_MirrorSentenceToUart2(const char *sentence)
{
#if GPS_DEBUG
  uint16_t sentence_len;

  if ((sentence == NULL) || (sentence[0] == '\0'))
  {
    return;
  }

  sentence_len = (uint16_t)strlen(sentence);

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)sentence, sentence_len, 20U);
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 20U);
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
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)csv_buf, (uint16_t)csv_len, 20U);
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
  gps_last_nmea_config_ms = 0U;

  memset((void *)&gps_data, 0, sizeof(gps_data));
  memset((void *)&gps_diag, 0, sizeof(gps_diag));
  gps_diag.pair001_last_result = -1;
  GPS_ResetRxState();

  gps_diag.init_count = 1U;
  gps_diag.active_baud_rate = gps_uart->Init.BaudRate;
  gps_diag.detected_baud_rate = gps_uart->Init.BaudRate;
  gps_diag.baud_locked = 1U;
  gps_diag.uart_rx_pin_level = (uint8_t)((((GPS_BASE_RX_GPIO_PORT)->IDR & GPS_BASE_RX_PIN) != 0U) ? 1U : 0U);
  gps_diag.uart_rx_pin_last_level = gps_diag.uart_rx_pin_level;

  if (gps_uart->Init.BaudRate != GPS_DEFAULT_BAUD_RATE)
  {
    gps_diag.start_receive_status = (uint8_t)HAL_ERROR;
    GPS_UpdateUartDiagState();
    return HAL_ERROR;
  }

  GPS_UpdateUartDiagState();

  /* Arm RX so bytes arriving during the restore reboot are captured */
  gps_diag.start_receive_calls++;
  {
    HAL_StatusTypeDef arm_status = GPS_RearmReceiveIT();
    if (arm_status != HAL_OK)
    {
      return arm_status;
    }
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

  /* Enable standard NMEA output and GGA/RMC rates */
  (void)GPS_SendCommand(GPS_ENABLE_NMEA_MODE_CMD);
  (void)GPS_SendCommand(GPS_QUERY_NMEA_MODE_CMD);
  (void)GPS_SendCommand(GPS_QUERY_PORT_UART1_CMD);
  (void)GPS_SendCommand(GPS_QUERY_PORT_UART2_CMD);
  (void)GPS_SendCommand(GPS_QUERY_PORT_UART3_CMD);
  (void)GPS_SendCommand(GPS_ENABLE_PORT_UART3_CMD);
  (void)GPS_SendCommand(GPS_QUERY_PORT_UART3_CMD);
  (void)GPS_SendCommand(GPS_ENABLE_ALL_NMEA_PAIR_CMD);
  (void)GPS_SendCommand(GPS_ENABLE_GGA_PAIR_CMD);
  (void)GPS_SendCommand(GPS_ENABLE_RMC_PAIR_CMD);
  (void)GPS_SendCommand(GPS_QUERY_ALL_NMEA_PAIR_CMD);
  (void)GPS_SendCommand(GPS_QUERY_GGA_PAIR_CMD);
  (void)GPS_SendCommand(GPS_QUERY_RMC_PAIR_CMD);
  (void)GPS_SendCommand(GPS_ENABLE_GGA_CMD);
  (void)GPS_SendCommand(GPS_ENABLE_RMC_CMD);
  (void)GPS_SendCommand(GPS_QUERY_UART_CMD);
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

  /* Save to flash so message rates take effect */
  (void)GPS_SendCommand(GPS_PAIR_SAVE_CMD);
  (void)GPS_SendCommand(GPS_SAVEPAR_CMD);
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

  status = GPS_RearmReceiveIT();

  return status;
}

void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((gps_uart == NULL) || (huart == NULL) || (huart->Instance != gps_uart->Instance))
  {
    return;
  }
  // Check if the Overrun Error flag is set
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
      // Clear the ORE flag to unfreeze the RX hardware
      __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
  }

  GPS_PushByte(gps_rx_byte);
  (void)GPS_RearmReceiveIT();
}

void GPS_Process(void)
{
  uint32_t now;

  GPS_SampleRxPinLevel();

  now = HAL_GetTick();

  if ((gps_uart != NULL)
      && (((gps_diag.gga_count == 0U) || (gps_diag.rmc_count == 0U)))
      && ((now - gps_last_nmea_config_ms) >= GPS_COMMAND_RETRY_MS))
  {
    (void)GPS_SendCommand(GPS_ENABLE_NMEA_MODE_CMD);
    (void)GPS_SendCommand(GPS_QUERY_PORT_UART1_CMD);
    (void)GPS_SendCommand(GPS_QUERY_PORT_UART2_CMD);
    (void)GPS_SendCommand(GPS_QUERY_PORT_UART3_CMD);
    (void)GPS_SendCommand(GPS_ENABLE_PORT_UART3_CMD);
    (void)GPS_SendCommand(GPS_QUERY_PORT_UART3_CMD);
    (void)GPS_SendCommand(GPS_ENABLE_ALL_NMEA_PAIR_CMD);
    (void)GPS_SendCommand(GPS_ENABLE_GGA_PAIR_CMD);
    (void)GPS_SendCommand(GPS_ENABLE_RMC_PAIR_CMD);
    (void)GPS_SendCommand(GPS_QUERY_ALL_NMEA_PAIR_CMD);
    (void)GPS_SendCommand(GPS_QUERY_GGA_PAIR_CMD);
    (void)GPS_SendCommand(GPS_QUERY_RMC_PAIR_CMD);
    (void)GPS_SendCommand(GPS_ENABLE_GGA_CMD);
    (void)GPS_SendCommand(GPS_ENABLE_RMC_CMD);
    (void)GPS_SendCommand(GPS_QUERY_UART_CMD);
    gps_last_nmea_config_ms = now;
  }

  if ((gps_uart != NULL)
      && (gps_diag.pqtmtar_count == 0U)
      && (gps_config_stage != GPS_CONFIG_STAGE_VERIFIED)
      && ((now - gps_last_command_ms) >= GPS_COMMAND_RETRY_MS))
  {
    if ((gps_config_stage == GPS_CONFIG_STAGE_ENABLE_WRITE)
        || (gps_config_stage == GPS_CONFIG_STAGE_WAIT_ENABLE_ACK))
    {
      if (GPS_SendCommand(GPS_PQTMTAR_ENABLE_CMD) == HAL_OK)
      {
        gps_config_stage = GPS_CONFIG_STAGE_WAIT_ENABLE_ACK;
        gps_last_command_ms = now;
      }
    }
    else
    {
      if (GPS_SendCommand(GPS_PQTMTAR_QUERY_CMD) == HAL_OK)
      {
        gps_diag.config_readback_status = (uint8_t)HAL_BUSY;
        gps_config_stage = GPS_CONFIG_STAGE_WAIT_READBACK;
        gps_last_command_ms = now;
      }
    }
  }

  GPS_ProcessPendingBytes();
}

static void GPS_ProcessPendingBytes(void)
{
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
