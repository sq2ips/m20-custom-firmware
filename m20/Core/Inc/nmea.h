#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include <stdint.h>

#define DATA_SIZE               35 // Max number of NMEA sentences in one parsing
#define SENTENCE_SIZE           82+1 // Max lenght of a NMEA sentence is 82 characters
#define MAX_SENTENCE_ELEMENTS   10 // Max number of NMEA sentence elements (no element with number bigger than 9 is used)
#define SENTENCE_ELEMENT_LEN    12 // Max lenght of a sentence element

#define GpsRxBuffer_SIZE 512

typedef struct TNMEADATA {
  float Lat;    // latitude in degrees with decimal places + for N - for S
  float Lon;    // longitude in degrees with decimal places
  uint16_t Alt; // altitude in meters
  uint8_t Speed;
  int16_t AscentRate; // positive (raising) or negative (falling) vertical speed value in cm/s
  uint8_t Hours;
  uint8_t Minutes;
  uint8_t Seconds;
  // float HDOP; //horizontal dilution of precision
  uint8_t Sats; // number of satellites used in measurement
  uint8_t Fix;  // 0 = no data, 1 = no fix, 2 = 2D fix, 3 = 3D fix
} NMEA;

void ParseNMEA(NMEA *nmea_data, uint8_t *buffer);

#endif /* INC_NMEA_H_ */
