/*
 *  xm_gps.c
 *  By SQ2IPS
 */

#include "xm_gps.h"
#include "config.h"
#include "main.h"

#include <string.h>

const uint8_t syncChain[] = {0xAA, 0xAA, 0xAA, 0x03};

float oldAlt = 0;

uint32_t oldTime = 0;

void ParseXM(XMDATA *GpsData, uint8_t *buffer, uint8_t position) {
  GpsData->Fix = *(uint8_t *)(buffer + 4 + position);

  int32_t Lat;
  memcpy(&Lat, buffer + 5 + position, sizeof(int32_t));
  Lat = ((Lat & 0xFF000000) >> 24) | ((Lat & 0x00FF0000) >> 8) |
        ((Lat & 0x0000FF00) << 8) | ((Lat & 0x000000FF) << 24);
  int32_t Lon;
  memcpy(&Lon, buffer + 9 + position, sizeof(int32_t));
  Lon = ((Lon & 0xFF000000) >> 24) | ((Lon & 0x00FF0000) >> 8) |
        ((Lon & 0x0000FF00) << 8) | ((Lon & 0x000000FF) << 24);

  uint32_t Alt;
  memcpy(&Alt, buffer + 12 + position, sizeof(uint32_t)); // from uint24_t
  Alt = ((Alt & 0xFF000000) >> 24) | ((Alt & 0x00FF0000) >> 8) |
        ((Alt & 0x0000FF00) << 8);

  GpsData->Lat = Lat / 1e6;
  GpsData->Lon = Lon / 1e6;
  GpsData->Alt = Alt / 1e2;

  memcpy(&GpsData->Time, buffer + 21 + position,
         sizeof(uint32_t)); // from uint24_t
  GpsData->Time = ((GpsData->Time & 0xFF000000) >> 24) |
                  ((GpsData->Time & 0x00FF0000) >> 8) |
                  ((GpsData->Time & 0x0000FF00) << 8);

  if (GpsData->Fix > 1 && GpsData->Alt != 0) {
    if (GpsData->Time != 0) {
      if (oldTime == 0) {
        oldAlt = GpsData->Alt;
        oldTime = GpsData->Time;
      }
      if ((GpsData->Time - oldTime) < 0) {
        GpsData->Time += 3600 * 24;
      }
      if ((GpsData->Time - oldTime) >= AscentRateTime) {
        GpsData->AscentRate =
            (float)(GpsData->Alt - oldAlt) / (GpsData->Time - oldTime);
        if ((GpsData->Time - oldTime) < 0) {
          GpsData->Time -= 3600 * 24;
        }
        oldAlt = GpsData->Alt;
        oldTime = GpsData->Time;
      }
    } else {
      oldTime = 0;
    }
  }

  GpsData->Hours = (GpsData->Time / 3600) % 24;
  GpsData->Minutes = (GpsData->Time / 60) % 60;
  GpsData->Seconds = GpsData->Time % 60;

  // memcpy(&GpsData->Sats, buffer + 27 + position, sizeof(uint8_t)); // not
  // sure of this byte meaning (always 12)
  for (GpsData->Sats = 0; GpsData->Sats < 16; GpsData->Sats++) {
    if (buffer[position + 28 + GpsData->Sats] == 0) {
      break;
    }
  }
}

int8_t getPosition(uint8_t *buffer) {
  for (uint8_t pos = 0; pos <= GPS_FRAME_LEN + 1; pos++) {
    bool sync = true;
    for (uint8_t syncCount = 0;
         syncCount < sizeof(syncChain) / sizeof(syncChain[0]); syncCount++) {
      if (buffer[pos + syncCount] != syncChain[syncCount]) {
        sync = false;
      }
    }
    if (sync) {
      return pos;
    }
  }
  return -1;
}

void parseXMframe(XMDATA *GpsData, uint8_t *buffer) {
  int8_t pos = getPosition(buffer);
  if (pos != -1) {
    ParseXM(GpsData, buffer, pos);
  } else {
    GpsData->Fix = 0;
  }
}
