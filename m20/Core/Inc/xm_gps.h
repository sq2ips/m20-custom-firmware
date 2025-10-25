#ifndef INC_XM_GPS_H_
#define INC_XM_GPS_H_

#include <stdint.h>

typedef struct TXMDATA {
  uint8_t
      Fix; // 00 = no data/parsing error, 01 = no fix, 02 = 2D fix, 03 = 3D fix
  float Lat;        // latitude in degrees with decimal places + for N - for S
  float Lon;        // longitude in degrees with decimal places
  uint16_t Alt;        // altitude in meters
  float AscentRate; // m/s
  float GroundSpeed;
  uint32_t Time;
  uint8_t Hours;
  uint8_t Minutes;
  uint8_t Seconds;
  uint8_t Sats; // number of satellites used in measurement
} XMDATA;

void parseXMframe(XMDATA *GpsData, uint8_t *buffer);

void incTimeCountGps();

#endif
