#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *gps_uart = NULL;
static uint8_t gps_rx_byte = 0U;
static volatile uint16_t gps_rx_head = 0U;
static volatile uint16_t gps_rx_tail = 0U;
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];

static char gps_line_buffer[GPS_LINE_BUFFER_SIZE];
static uint16_t gps_line_index = 0U;

volatile GPS_Data_t gps_data = {0};
volatile uint32_t gps_rx_count = 0U;
volatile uint8_t gps_last_byte = 0U;
volatile char gps_last_sentence[GPS_LINE_BUFFER_SIZE] = {0};
volatile uint8_t gps_sentence_ready = 0U;
volatile uint32_t gps_sentence_count = 0U;
volatile uint32_t gps_rmc_count = 0U;
volatile uint32_t gps_gga_count = 0U;
volatile uint32_t gps_vtg_count = 0U;

static uint16_t GPS_NextIndex(uint16_t idx)
{
  return (uint16_t)((idx + 1U) % GPS_RX_BUFFER_SIZE);
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

  gps_rmc_count++;
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

  gps_gga_count++;
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

  gps_vtg_count++;
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
}

HAL_StatusTypeDef GPS_Init(UART_HandleTypeDef *uart)
{
  if (uart == NULL)
  {
    return HAL_ERROR;
  }

  gps_uart = uart;
  gps_rx_head = 0U;
  gps_rx_tail = 0U;
  gps_line_index = 0U;
  gps_sentence_ready = 0U;

  memset((void *)&gps_data, 0, sizeof(gps_data));
  memset((void *)gps_last_sentence, 0, sizeof(gps_last_sentence));

  gps_rx_count = 0U;
  gps_last_byte = 0U;
  gps_sentence_count = 0U;
  gps_rmc_count = 0U;
  gps_gga_count = 0U;
  gps_vtg_count = 0U;

  return HAL_OK;
}

HAL_StatusTypeDef GPS_StartReceiveIT(void)
{
  if (gps_uart == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive_IT(gps_uart, &gps_rx_byte, 1U);
}

void GPS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((gps_uart == NULL) || (huart == NULL) || (huart->Instance != gps_uart->Instance))
  {
    return;
  }

  gps_last_byte = gps_rx_byte;
  gps_rx_count++;

  {
    uint16_t next = GPS_NextIndex(gps_rx_head);

    if (next != gps_rx_tail)
    {
      gps_rx_buffer[gps_rx_head] = gps_rx_byte;
      gps_rx_head = next;
    }
  }

  (void)GPS_StartReceiveIT();
}

void GPS_Process(void)
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

        snprintf((char *)gps_last_sentence,
                 GPS_LINE_BUFFER_SIZE,
                 "%s",
                 gps_line_buffer);

        gps_sentence_ready = 1U;
        gps_sentence_count++;
        gps_line_index = 0U;

        GPS_HandleSentence((const char *)gps_last_sentence);
        gps_sentence_ready = 0U;
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
