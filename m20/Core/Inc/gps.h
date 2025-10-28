#ifndef INC_GPS_H
#define INC_GPS_H

#include <stdint.h>

typedef struct {
    uint8_t Fix;          // 0 = no data, 1 = no fix, 2 = 2D fix, 3 = 3D fix
    uint8_t Sats;         // number of satellites used in measurement
    float Lat;            // latitude in degrees with decimal places + for N - for S
    float Lon;            // longitude in degrees with decimal places
    uint16_t Alt;         // altitude in meters
    int16_t AscentRate;   // positive (raising) or negative (falling) vertical speed value in cm/s
    uint8_t Speed;        // speed over ground in m/s
    uint8_t Hours;        // time in hours
    uint8_t Minutes;      // time in minutes
    uint8_t Seconds;      // time in seconds
} GPS;

void gps_debug(GPS GpsData);

#endif // INC_GPS_H
