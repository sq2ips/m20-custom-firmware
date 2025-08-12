#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include <stdint.h>

typedef struct TNMEADATA {
  float Lat;    // latitude in degrees with decimal places + for N - for S
  float Lon;    // longitude in degrees with decimal places
  uint16_t Alt; // altitude in meters
  uint16_t Speed;
  float AscentRate;
  uint8_t Hours;
  uint8_t Minutes;
  uint8_t Seconds;
  // float HDOP; //horizontal dilution of precision
  uint8_t Sats; // number of satellites used in measurement
  uint8_t Fix;  // 0 = no data, 1 = no fix, 2 = 2D fix, 3 = 3D fix
  uint8_t Corr; // number of correct frames
} NMEA;

void ParseNMEA(NMEA *nmea_data, uint8_t *buffer);

#endif /* INC_NMEA_H_ */
