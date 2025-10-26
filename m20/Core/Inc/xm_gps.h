#ifndef INC_XM_GPS_H_
#define INC_XM_GPS_H_

#include <stdint.h>

#define GPS_FRAME_LEN 62             // Length of XM1110 (type 2) frame
#define GpsRxBuffer_SIZE GPS_FRAME_LEN * 2

typedef struct TXMDATA {
  uint8_t
      Fix; // 00 = no data/parsing error, 01 = no fix, 02 = 2D fix, 03 = 3D fix
  float Lat;        // latitude in degrees with decimal places + for N - for S
  float Lon;        // longitude in degrees with decimal places
  uint16_t Alt;     // altitude in meters
  int16_t AscentRate; // positive (raising) or negative (falling) vertical speed value in cm/s
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
