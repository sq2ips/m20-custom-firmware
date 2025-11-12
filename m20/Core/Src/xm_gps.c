/*
 * xm_gps.c
 * By SQ2IPS & SP2IML
 *  
 * Example real data frames
 *  1. indoor
 * AA AA AA 03 | 01  | 05 5D 4A 7F | 00 00 00 00 | 00 3A 98 | 00 00   | 00 00   | 00 00   | 05 45 DC | 00 | 13 | 12 | 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 | 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 | 75 6D
 * 2. first outdoor frame
 * AA AA AA 03 | 01  | 05 5D 4A 7F | 00 00 00 00 | 00 3A 98 | 00 00   | 00 00   | 00 00   | 05 45 B8 | 00 | 13 | 12 | 28 2A 00 00 00 00 00 00 00 00 00 00 00 00 00 00 | 12 19 00 00 00 00 00 00 00 00 00 00 00 00 00 00 | 9A 80
 * 3. last outdoor frame
 * AA AA AA 03 | 03  | 03 40 58 28 | 01 1A FA 13 | 00 1A 31 | FF E6   | FF BC   | 00 01   | 07 AE D0 | 01 | 57 | 12 | 11 16 22 28 1C 24 20 20 14 1F 00 00 00 00 00 00 | 04 05 10 12 15 19 1A 1C 1D 1F 00 00 00 00 00 00 | 24 1C
 * offset      | 0   | 1           | 5           | 9        | 12      | 14      | 16      | 18       | 21 | 22 | 23 | 24                                              | 40                                              | 56
 * preambule   | fix | latitude    | longitude   | alt      | lat dir | lon dir | alt dir | gps time | ?  | ?  | 12 | satelites s/n ratio                             | satelites names (numbers)                       | checksum
 */

#include "config.h"
#include "xm_gps.h"
#include "utils.h"

#ifdef GPS_DEBUG
#include <stdio.h>
#endif

#define XM_PREAMBULE_LEN 4
const uint8_t preambule[XM_PREAMBULE_LEN] = {0xAA, 0xAA, 0xAA, 0x03};

float oldAlt = 0;

uint32_t oldTime = 0;

uint32_t convert_buffer_to_uint32(const uint8_t *buffer, const uint8_t size) {
  uint32_t result = 0;
  for (int i = 0; i < size; i++) {
    result <<= 8;
    result |= buffer[i];
  }
  return result;
}

int16_t timeDifference(uint32_t time1, uint32_t time2) {
  if (time1 < time2)
    return time2 - time1;
  return 24 * 60 * 60 - (time1 - time2);
}

int16_t calculateAscentRate(uint16_t alt1, uint16_t alt2, uint32_t time1, uint32_t time2) {
  const int16_t altDiff = alt2 - alt1;
  const int16_t timeDiff = timeDifference(time1, time2);

  return (int16_t)Round((float)altDiff / timeDiff * 100);
}

void ParseXM(GPS *GpsData, const uint8_t *buffer, const uint8_t frameStartPosition) {
  const uint8_t pos = frameStartPosition + XM_PREAMBULE_LEN;
  const uint8_t FIX_OFFSET = pos + 0;
  const uint8_t LAT_OFFSET = pos + 1;
  const uint8_t LON_OFFSET = pos + 5;
  const uint8_t ALT_OFFSET = pos + 9;
  const uint8_t LAT_DIR_OFFSET = pos + 12;
  const uint8_t LON_DIR_OFFSET = pos + 14;
  const uint8_t ALT_DIR_OFFSET = pos + 16;
  const uint8_t TIME_OFFSET = pos + 18;
  const uint8_t SATS_OFFSET = pos + 24;
  const uint8_t MAX_SATS = 16;

  #ifdef GPS_DEBUG
  printf("XM GPS raw: ");
  for (int i = frameStartPosition; i < frameStartPosition + GPS_FRAME_LEN; i++) {
    printf("%02X ", buffer[i]);
  }
  printf("\r\n");
  #endif

  GpsData->Fix = buffer[FIX_OFFSET];

  const uint32_t Lat = convert_buffer_to_uint32(buffer + LAT_OFFSET, 4);
  const uint32_t Lon = convert_buffer_to_uint32(buffer + LON_OFFSET, 4);
  const uint32_t Alt = convert_buffer_to_uint32(buffer + ALT_OFFSET, 3);

  GpsData->Lat = (float)(Lat / 1e6);    // converts from microdegrees to degrees
  GpsData->Lon = (float)(Lon / 1e6);    // converts from microdegrees to degrees
  GpsData->Alt = (uint16_t)(Alt / 100); // converts from centimeters to meters

  const uint32_t Time = convert_buffer_to_uint32(buffer + TIME_OFFSET, 3); // number of seconds from midnight

  GpsData->Hours = (Time / 3600) % 24;
  GpsData->Minutes = (Time / 60) % 60;
  GpsData->Seconds = Time % 60;

  if (GpsData->Fix == 3 && GpsData->Alt > 0) {
    if (Time > 0) {
      if (oldTime == 0) {
        oldAlt = GpsData->Alt;
        oldTime = Time;
      } else
        if (timeDifference(oldTime, Time) > AscentRateTime)
          GpsData->AscentRate = calculateAscentRate(oldAlt, GpsData->Alt, oldTime, Time);
    } else
      oldTime = 0;
  }

  uint8_t sats = 0;
  while (buffer[SATS_OFFSET + sats] != 0 && sats < MAX_SATS)
    sats++;
  GpsData->Sats = sats;
}

int8_t getFrameStartPosition(const uint8_t *buffer) {
  for (int8_t fpos = 0; fpos <= GPS_FRAME_LEN + XM_PREAMBULE_LEN; fpos++) {
    if (buffer[fpos] == preambule[0]) {
      if (buffer[fpos + 1] == preambule[1] &&
          buffer[fpos + 2] == preambule[2] &&
          buffer[fpos + 3] == preambule[3]) {
        return fpos;
      }
    }
  }
  return -1;
}

void parseXMframe(GPS *GpsData, const uint8_t *buffer) {
  const int8_t frameStartPosition = getFrameStartPosition(buffer);
  if (frameStartPosition != -1)
    ParseXM(GpsData, buffer, frameStartPosition);
  else
    GpsData->Fix = 0;
}
