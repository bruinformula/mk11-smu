#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GPS_DEFAULT_BAUD_RATE       460800U
#define GPS_PQTMTAR_ENABLE_CMD      "$PQTMCFGMSGRATE,W,PQTMTAR,1,1*09\r\n"
#define GPS_COMMAND_RETRY_MS        1000U

static UART_HandleTypeDef *gps_uart = NULL;
static uint8_t gps_rx_byte = 0U;
static volatile uint16_t gps_rx_head = 0U;
static volatile uint16_t gps_rx_tail = 0U;
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];

static char gps_line_buffer[GPS_LINE_BUFFER_SIZE];
static uint16_t gps_line_index = 0U;
static uint8_t gps_heading_command_sent = 0U;
static uint32_t gps_last_command_ms = 0U;

volatile GPS_Data_t gps_data = {0};
volatile GPS_Diag_t gps_diag = {0};

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
  uint8_t level = (uint8_t)(((GPIOB->IDR & GPIO_PIN_7) != 0U) ? 1U : 0U);

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
    if (strstr(sentence, ",OK") != NULL)
    {
      gps_diag.heading_message_enabled = 1U;
    }
    else if (strstr(sentence, ",ERROR") != NULL)
    {
      gps_diag.heading_message_enabled = 0U;
    }
  }
}

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart)
{
  if (uart == NULL)
  {
    return HAL_ERROR;
  }

  gps_uart = uart;
  gps_heading_command_sent = 0U;
  gps_last_command_ms = 0U;

  memset((void *)&gps_data, 0, sizeof(gps_data));
  memset((void *)&gps_diag, 0, sizeof(gps_diag));
  GPS_ResetRxState();

  gps_diag.init_count = 1U;
  gps_diag.active_baud_rate = gps_uart->Init.BaudRate;
  gps_diag.detected_baud_rate = gps_uart->Init.BaudRate;
  gps_diag.baud_locked = 1U;
  gps_diag.uart_rx_pin_level = (uint8_t)(((GPIOB->IDR & GPIO_PIN_7) != 0U) ? 1U : 0U);
  gps_diag.uart_rx_pin_last_level = gps_diag.uart_rx_pin_level;

  if (gps_uart->Init.BaudRate != GPS_DEFAULT_BAUD_RATE)
  {
    gps_diag.start_receive_status = (uint8_t)HAL_ERROR;
    GPS_UpdateUartDiagState();
    return HAL_ERROR;
  }

  GPS_UpdateUartDiagState();

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

  if ((status == HAL_OK) && (gps_heading_command_sent == 0U))
  {
    if (GPS_SendCommand(GPS_PQTMTAR_ENABLE_CMD) == HAL_OK)
    {
      gps_heading_command_sent = 1U;
      gps_last_command_ms = HAL_GetTick();
    }
  }

  return status;
}

void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((gps_uart == NULL) || (huart == NULL) || (huart->Instance != gps_uart->Instance))
  {
    return;
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
      && (gps_diag.rx_count == 0U)
      && (gps_heading_command_sent != 0U)
      && ((now - gps_last_command_ms) >= GPS_COMMAND_RETRY_MS))
  {
    if (GPS_SendCommand(GPS_PQTMTAR_ENABLE_CMD) == HAL_OK)
    {
      gps_last_command_ms = now;
    }
  }

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
