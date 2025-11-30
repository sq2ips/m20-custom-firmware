#include "gps.h"

#if GPS_DEBUG

#include <stdio.h>

void gps_debug(const GPS GpsData)
{
	printf("GPS DEBUG: %02d:%02d:%02d Fix: %d, Sats: %d, Lat: %ld, Lon: %ld, Alt: %d m, Ascent rate: %d cm/s, Speed: %d km/h\r\n", GpsData.Hours,
	       GpsData.Minutes, GpsData.Seconds, GpsData.Fix, GpsData.Sats, (int32_t)(GpsData.Lat * 1e6), (int32_t)(GpsData.Lon * 1e6), GpsData.Alt,
	       GpsData.AscentRate, GpsData.Speed);
}

#endif
