#if DEBUG

#include "gps.h"
#include <stdio.h>

void gps_debug(const GPS GpsData) {
  printf(
    "Fix: %d, Sats: %d, "
    "Lat: %d, Lon: %d, Alt: %d "
    "AscentRate: %d cm/s, Speed: %d m/s, "
    "H:M:S: %d:%d:%d\r\n",
    GpsData.Fix, GpsData.Sats,
    (int32_t)(GpsData.Lat * 1e6), (int32_t)(GpsData.Lon * 1e6), GpsData.Alt,
    GpsData.AscentRate, GpsData.Speed,
    GpsData.Hours, GpsData.Minutes, GpsData.Seconds
  );
}

#endif
