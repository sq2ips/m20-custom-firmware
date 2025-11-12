#ifndef INC_XM_GPS_H_
#define INC_XM_GPS_H_

#include "gps.h"
#include <stdint.h>

#define GPS_FRAME_LEN 62             // Length of XM1110 (type 2) frame
#define GpsRxBuffer_SIZE (GPS_FRAME_LEN * 2)

void parseXMframe(GPS *GpsData, const uint8_t *buffer);
void incTimeCountGps();

#endif
