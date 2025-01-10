/*
 * nmea.h
 *
 *  Created on: Dec 30, 2024
 *      Author: pawel
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "main.h"
#include "config.h"

typedef struct NMEA_DATA {
    float Lat; //latitude in degrees with decimal places + for N - for S
    float Lon; //longitude in degrees with decimal places
    float Alt; //altitude in meters
    float Speed;
    float AscentRate;
    uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;
    float HDOP; //horizontal dilution of precision
    uint8_t Sats; //number of satellites used in measurement
    bool Fix; // 1 = fix, 0 = no fix
    uint8_t Corr; // number of correct frames
} NMEA;

void ParseNMEA(NMEA *nmea_data, uint8_t *buffer);

#endif /* INC_NMEA_H_ */


