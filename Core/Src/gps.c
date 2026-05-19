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
static uint8_t gps_in_ascii_sentence = 0U;

volatile GPS_Data_t gps_data = {0};
volatile uint32_t gps_rx_count = 0U;
volatile uint8_t gps_last_byte = 0U;
volatile char gps_last_sentence[GPS_LINE_BUFFER_SIZE] = {0};
volatile uint8_t gps_sentence_ready = 0U;
volatile uint32_t gps_sentence_count = 0U;
volatile uint32_t gps_rmc_count = 0U;
volatile uint32_t gps_gga_count = 0U;
volatile uint32_t gps_vtg_count = 0U;

/* Binary-tolerant parser debug counters */
volatile uint32_t gps_dollar_count = 0U;
volatile uint32_t gps_ascii_sentence_count = 0U;
volatile uint32_t gps_binary_drop_count = 0U;
volatile uint32_t gps_nonascii_sentence_drop_count = 0U;
volatile uint32_t gps_line_overflow_count = 0U;

/* PQTMTAR / moving-base heading debug */
volatile uint32_t gps_pqtmtar_count = 0U;
volatile uint8_t gps_pqtmtar_status = 0U;
volatile float gps_pqtmtar_heading_deg = 0.0f;
volatile float gps_pqtmtar_pitch_deg = 0.0f;
volatile float gps_pqtmtar_roll_deg = 0.0f;
volatile float gps_pqtmtar_baseline_m = 0.0f;
volatile uint8_t gps_pqtmtar_msgver = 0U;
volatile float gps_pqtmtar_utc_time = 0.0f;
volatile uint8_t gps_pqtmtar_quality = 0U;
volatile float gps_pqtmtar_pitch_accuracy_deg = 0.0f;
volatile float gps_pqtmtar_heading_accuracy_deg = 0.0f;
volatile uint8_t gps_pqtmtar_used_sv = 0U;

/* PQTMTAR solution-valid debug */
volatile uint8_t gps_pqtmtar_solution_valid = 0U;
volatile uint32_t gps_pqtmtar_valid_count = 0U;
volatile uint32_t gps_pqtmtar_invalid_count = 0U;
volatile uint32_t gps_pqtmtar_last_valid_ms = 0U;
volatile uint32_t gps_pqtmtar_last_invalid_ms = 0U;

/* PQTM command response debug */
volatile char gps_last_pqtm_response[GPS_LINE_BUFFER_SIZE] = {0};

volatile uint32_t gps_pqtmcfgprot_ok_count = 0U;
volatile uint32_t gps_pqtmcfgprot_error_count = 0U;

volatile uint32_t gps_pqtmcfgmsgrate_ok_count = 0U;
volatile uint32_t gps_pqtmcfgmsgrate_error_count = 0U;

volatile uint32_t gps_pqtmsavepar_ok_count = 0U;
volatile uint32_t gps_pqtmsavepar_error_count = 0U;

volatile uint32_t gps_pqtm_response_other_count = 0U;

/* Raw fields for debugging until final field meanings are confirmed */
volatile float gps_pqtmtar_f1 = 0.0f;
volatile float gps_pqtmtar_f2 = 0.0f;
volatile float gps_pqtmtar_f3 = 0.0f;
volatile float gps_pqtmtar_f4 = 0.0f;
volatile float gps_pqtmtar_f5 = 0.0f;
volatile float gps_pqtmtar_f6 = 0.0f;
volatile float gps_pqtmtar_f7 = 0.0f;
volatile float gps_pqtmtar_f8 = 0.0f;

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

static void GPS_ParsePQTMTAR(const char *sentence)
{
  char buffer[GPS_LINE_BUFFER_SIZE];
  char *fields[24] = {0};
  uint32_t field_count;

  snprintf(buffer, sizeof(buffer), "%s", sentence);
  field_count = GPS_SplitFields(buffer, fields, 24U);

  /*
   * We are intentionally storing raw fields first.
   * This avoids guessing the exact PQTMTAR field meanings incorrectly.
   *
   * fields[0] = "$PQTMTAR"
   * fields[1] = first payload field
   * fields[2] = second payload field
   * ...
   */
  if (field_count < 2U)
  {
    return;
  }

  gps_pqtmtar_f1 = (field_count > 1U && fields[1][0] != '\0') ? (float)atof(fields[1]) : 0.0f;
  gps_pqtmtar_f2 = (field_count > 2U && fields[2][0] != '\0') ? (float)atof(fields[2]) : 0.0f;
  gps_pqtmtar_f3 = (field_count > 3U && fields[3][0] != '\0') ? (float)atof(fields[3]) : 0.0f;
  gps_pqtmtar_f4 = (field_count > 4U && fields[4][0] != '\0') ? (float)atof(fields[4]) : 0.0f;
  gps_pqtmtar_f5 = (field_count > 5U && fields[5][0] != '\0') ? (float)atof(fields[5]) : 0.0f;
  gps_pqtmtar_f6 = (field_count > 6U && fields[6][0] != '\0') ? (float)atof(fields[6]) : 0.0f;
  gps_pqtmtar_f7 = (field_count > 7U && fields[7][0] != '\0') ? (float)atof(fields[7]) : 0.0f;
  gps_pqtmtar_f8 = (field_count > 8U && fields[8][0] != '\0') ? (float)atof(fields[8]) : 0.0f;

  /*
   * PQTMTAR field order:
   * fields[0]  = "$PQTMTAR"
   * fields[1]  = MsgVer
   * fields[2]  = UTC time, hhmmss.sss
   * fields[3]  = Quality
   * fields[4]  = Res1, reserved, normally empty
   * fields[5]  = Baseline length, meters
   * fields[6]  = Pitch, degrees
   * fields[7]  = Res2, reserved, normally empty
   * fields[8]  = Heading, degrees
   * fields[9]  = Pitch accuracy, degrees
   * fields[10] = Res3, reserved, normally empty
   * fields[11] = Heading accuracy, degrees
   * fields[12] = UsedSV, may include checksum suffix like "21*59"
   */

  gps_pqtmtar_msgver =
      (field_count > 1U && fields[1][0] != '\0') ? (uint8_t)atoi(fields[1]) : 0U;

  gps_pqtmtar_utc_time =
      (field_count > 2U && fields[2][0] != '\0') ? (float)atof(fields[2]) : 0.0f;

  gps_pqtmtar_quality =
      (field_count > 3U && fields[3][0] != '\0') ? (uint8_t)atoi(fields[3]) : 0U;

  /* Keep old name as alias for now so existing Live Expressions still work. */
  gps_pqtmtar_status = gps_pqtmtar_quality;

  gps_pqtmtar_baseline_m =
      (field_count > 5U && fields[5][0] != '\0') ? (float)atof(fields[5]) : 0.0f;

  gps_pqtmtar_pitch_deg =
      (field_count > 6U && fields[6][0] != '\0') ? (float)atof(fields[6]) : 0.0f;

  gps_pqtmtar_roll_deg = 0.0f;

  gps_pqtmtar_heading_deg =
      (field_count > 8U && fields[8][0] != '\0') ? (float)atof(fields[8]) : 0.0f;

  gps_pqtmtar_pitch_accuracy_deg =
      (field_count > 9U && fields[9][0] != '\0') ? (float)atof(fields[9]) : 0.0f;

  gps_pqtmtar_heading_accuracy_deg =
      (field_count > 11U && fields[11][0] != '\0') ? (float)atof(fields[11]) : 0.0f;

  gps_pqtmtar_used_sv =
      (field_count > 12U && fields[12][0] != '\0') ? (uint8_t)atoi(fields[12]) : 0U;

  /*
   * Treat attitude as valid only when the module reports nonzero quality and
   * actually provides the moving-base fields. When invalid, PQTMTAR can still
   * be emitted, but Length/Pitch/Heading/UsedSV will be empty or zero.
   */
  if ((gps_pqtmtar_quality != 0U) &&
      (field_count > 8U) &&
      (fields[5][0] != '\0') &&
      (fields[8][0] != '\0'))
  {
    gps_pqtmtar_solution_valid = 1U;
    gps_pqtmtar_valid_count++;
    gps_pqtmtar_last_valid_ms = HAL_GetTick();
  }
  else
  {
    gps_pqtmtar_solution_valid = 0U;
    gps_pqtmtar_invalid_count++;
    gps_pqtmtar_last_invalid_ms = HAL_GetTick();
  }

  gps_pqtmtar_count++;
}

static void GPS_ParsePQTMResponse(const char *sentence)
{
  if (sentence == NULL)
  {
    return;
  }

  snprintf((char *)gps_last_pqtm_response,
           GPS_LINE_BUFFER_SIZE,
           "%s",
           sentence);

  if (strncmp(sentence, "$PQTMCFGPROT", strlen("$PQTMCFGPROT")) == 0)
  {
    if (strstr(sentence, ",OK") != NULL)
    {
      gps_pqtmcfgprot_ok_count++;
    }
    else if (strstr(sentence, ",ERROR") != NULL)
    {
      gps_pqtmcfgprot_error_count++;
    }
    else
    {
      gps_pqtm_response_other_count++;
    }
  }
  else if (strncmp(sentence, "$PQTMCFGMSGRATE", strlen("$PQTMCFGMSGRATE")) == 0)
  {
    if (strstr(sentence, ",OK") != NULL)
    {
      gps_pqtmcfgmsgrate_ok_count++;
    }
    else if (strstr(sentence, ",ERROR") != NULL)
    {
      gps_pqtmcfgmsgrate_error_count++;
    }
    else
    {
      gps_pqtm_response_other_count++;
    }
  }
  else if (strncmp(sentence, "$PQTMSAVEPAR", strlen("$PQTMSAVEPAR")) == 0)
  {
    if (strstr(sentence, ",OK") != NULL)
    {
      gps_pqtmsavepar_ok_count++;
    }
    else if (strstr(sentence, ",ERROR") != NULL)
    {
      gps_pqtmsavepar_error_count++;
    }
    else
    {
      gps_pqtm_response_other_count++;
    }
  }
  else
  {
    gps_pqtm_response_other_count++;
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
  else if (strncmp(sentence, "$PQTMTAR", strlen("$PQTMTAR")) == 0)
  {
    GPS_ParsePQTMTAR(sentence);
  }
  else if ((strncmp(sentence, "$PQTMCFGPROT", strlen("$PQTMCFGPROT")) == 0) ||
           (strncmp(sentence, "$PQTMCFGMSGRATE", strlen("$PQTMCFGMSGRATE")) == 0) ||
           (strncmp(sentence, "$PQTMSAVEPAR", strlen("$PQTMSAVEPAR")) == 0))
  {
    GPS_ParsePQTMResponse(sentence);
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
  gps_in_ascii_sentence = 0U;
  gps_sentence_ready = 0U;

  memset((void *)&gps_data, 0, sizeof(gps_data));
  memset((void *)gps_last_sentence, 0, sizeof(gps_last_sentence));
  memset(gps_line_buffer, 0, sizeof(gps_line_buffer));

  gps_rx_count = 0U;
  gps_last_byte = 0U;
  gps_sentence_count = 0U;
  gps_rmc_count = 0U;
  gps_gga_count = 0U;
  gps_vtg_count = 0U;

  gps_dollar_count = 0U;
  gps_ascii_sentence_count = 0U;
  gps_binary_drop_count = 0U;
  gps_nonascii_sentence_drop_count = 0U;
  gps_line_overflow_count = 0U;

  gps_pqtmtar_count = 0U;
  gps_pqtmtar_status = 0U;
  gps_pqtmtar_heading_deg = 0.0f;
  gps_pqtmtar_pitch_deg = 0.0f;
  gps_pqtmtar_roll_deg = 0.0f;
  gps_pqtmtar_baseline_m = 0.0f;

  gps_pqtmtar_f1 = 0.0f;
  gps_pqtmtar_f2 = 0.0f;
  gps_pqtmtar_f3 = 0.0f;
  gps_pqtmtar_f4 = 0.0f;
  gps_pqtmtar_f5 = 0.0f;
  gps_pqtmtar_f6 = 0.0f;
  gps_pqtmtar_f7 = 0.0f;
  gps_pqtmtar_f8 = 0.0f;

  gps_pqtmtar_msgver = 0U;
  gps_pqtmtar_utc_time = 0.0f;
  gps_pqtmtar_quality = 0U;
  gps_pqtmtar_pitch_accuracy_deg = 0.0f;
  gps_pqtmtar_heading_accuracy_deg = 0.0f;
  gps_pqtmtar_used_sv = 0U;

  gps_pqtmtar_solution_valid = 0U;
  gps_pqtmtar_valid_count = 0U;
  gps_pqtmtar_invalid_count = 0U;
  gps_pqtmtar_last_valid_ms = 0U;
  gps_pqtmtar_last_invalid_ms = 0U;

  memset((void *)gps_last_pqtm_response, 0, sizeof(gps_last_pqtm_response));

  gps_pqtmcfgprot_ok_count = 0U;
  gps_pqtmcfgprot_error_count = 0U;

  gps_pqtmcfgmsgrate_ok_count = 0U;
  gps_pqtmcfgmsgrate_error_count = 0U;

  gps_pqtmsavepar_ok_count = 0U;
  gps_pqtmsavepar_error_count = 0U;

  gps_pqtm_response_other_count = 0U;

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
    uint8_t b = gps_rx_buffer[gps_rx_tail];
    gps_rx_tail = GPS_NextIndex(gps_rx_tail);

    /*
     * Binary-tolerant mode:
     * Ignore everything until a '$' is seen.
     * Real NMEA/PQTM ASCII sentences start with '$'.
     */
    if (b == (uint8_t)'$')
    {
      gps_dollar_count++;

      gps_line_index = 0U;
      gps_line_buffer[gps_line_index++] = (char)b;
      gps_in_ascii_sentence = 1U;
      continue;
    }

    /*
     * If we are not inside a '$...' sentence, this byte is binary/proprietary
     * traffic from the current GPS2 UART path. Drop it silently.
     */
    if (gps_in_ascii_sentence == 0U)
    {
      gps_binary_drop_count++;
      continue;
    }

    /*
     * End of ASCII sentence. Accept CR or LF. This handles both "\r\n" and
     * "\n" line endings.
     */
    if ((b == (uint8_t)'\r') || (b == (uint8_t)'\n'))
    {
      if (gps_line_index > 1U)
      {
        gps_line_buffer[gps_line_index] = '\0';

        snprintf((char *)gps_last_sentence,
                 GPS_LINE_BUFFER_SIZE,
                 "%s",
                 gps_line_buffer);

        gps_sentence_ready = 1U;
        gps_sentence_count++;
        gps_ascii_sentence_count++;

        GPS_HandleSentence((const char *)gps_last_sentence);

        gps_sentence_ready = 0U;
      }

      gps_line_index = 0U;
      gps_in_ascii_sentence = 0U;
      continue;
    }

    /*
     * Inside a '$...' sentence, only allow printable ASCII.
     * If binary appears after '$', discard that candidate sentence.
     */
    if ((b < 0x20U) || (b > 0x7EU))
    {
      gps_nonascii_sentence_drop_count++;
      gps_line_index = 0U;
      gps_in_ascii_sentence = 0U;
      continue;
    }

    if (gps_line_index < (GPS_LINE_BUFFER_SIZE - 1U))
    {
      gps_line_buffer[gps_line_index++] = (char)b;
    }
    else
    {
      gps_line_overflow_count++;
      gps_line_index = 0U;
      gps_in_ascii_sentence = 0U;
    }
  }
}
