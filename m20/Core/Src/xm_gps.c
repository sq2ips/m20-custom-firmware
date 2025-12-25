/*
 * xm_gps.c
 * By SQ2IPS & SP2IML
 *
 * Example real data frames
 *  1. indoor
 * AA AA AA 03 | 01  | 05 5D 4A 7F | 00 00 00 00 | 00 3A 98 | 00 00   | 00 00   | 00 00   | 05 45 DC | 00 | 13 | 12 | 00 00 00 00 00 00 00 00 00 00 00
 * 00 00 00 00 00 | 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 | 75 6D
 * 2. first outdoor frame
 * AA AA AA 03 | 01  | 05 5D 4A 7F | 00 00 00 00 | 00 3A 98 | 00 00   | 00 00   | 00 00   | 05 45 B8 | 00 | 13 | 12 | 28 2A 00 00 00 00 00 00 00 00 00
 * 00 00 00 00 00 | 12 19 00 00 00 00 00 00 00 00 00 00 00 00 00 00 | 9A 80
 * 3. last outdoor frame
 * AA AA AA 03 | 03  | 03 40 58 28 | 01 1A FA 13 | 00 1A 31 | FF E6   | FF BC   | 00 01   | 07 AE D0 | 01 | 57 | 12 | 11 16 22 28 1C 24 20 20 14 1F 00
 * 00 00 00 00 00 | 04 05 10 12 15 19 1A 1C 1D 1F 00 00 00 00 00 00 | 24 1C offset      | 0   | 1           | 5           | 9        | 12      | 14 |
 * 16      | 18       | 21 | 22 | 23 | 24                                              | 40                                              | 56
 * preambule   | fix | latitude    | longitude   | alt      | lat dir | lon dir | alt dir | gps time | ?  | ?  | 12 | satelites s/n ratio | satelites
 * names (numbers)                       | checksum
 */

#include "xm_gps.h"

#include "config.h"
#include "utils.h"

#ifdef GPS_DEBUG
#include <stdio.h>
#endif

#define XM_PREAMBULE_LEN 4
const uint8_t preambule[XM_PREAMBULE_LEN] = {0xAA, 0xAA, 0xAA, 0x03};

typedef struct {
  uint8_t Fix;
  uint32_t Lat;
  uint32_t Lon;
  uint32_t CurrentAlt;
  uint32_t PreviousAlt;
  uint32_t CurrentTime;
  uint32_t PreviousTime;
  uint8_t Sats;
} XmFrame;

XmFrame Xm;

void ParseXmFrame(const uint8_t* buffer, const uint8_t frameStartPosition) {
	const uint8_t pos = frameStartPosition + XM_PREAMBULE_LEN;
	const uint8_t FIX_OFFSET = pos + 0;
	const uint8_t LAT_OFFSET = pos + 1;
	const uint8_t LON_OFFSET = pos + 5;
	const uint8_t ALT_OFFSET = pos + 9;
	// const uint8_t LAT_DIR_OFFSET = pos + 12;
	// const uint8_t LON_DIR_OFFSET = pos + 14;
	// const uint8_t ALT_DIR_OFFSET = pos + 16;
	const uint8_t TIME_OFFSET = pos + 18;
	const uint8_t SATS_OFFSET = pos + 24;
	const uint8_t MAX_SATS = 16;

#if GPS_RAW_DEBUG
	printf("XM GPS raw frame: ");
	for (int i = frameStartPosition; i < frameStartPosition + GPS_FRAME_LEN; i++) {
		printf("%02X ", buffer[i]);
	}
	printf("\r\n");
#endif

	Xm.Fix = buffer[FIX_OFFSET];
	Xm.Lat = convert_buffer_to_uint32(buffer + LAT_OFFSET, 4);
	Xm.Lon = convert_buffer_to_uint32(buffer + LON_OFFSET, 4);
	Xm.CurrentAlt = convert_buffer_to_uint32(buffer + ALT_OFFSET, 3);   // in centimeters
	Xm.CurrentTime = convert_buffer_to_uint32(buffer + TIME_OFFSET, 3); // number of seconds from midnight

	Xm.Sats = 0;
	while (buffer[SATS_OFFSET + Xm.Sats] != 0 && Xm.Sats < MAX_SATS)
		Xm.Sats++;
}

void ConvertXmToGpsData(GPS* GpsData) {
	GpsData->Fix = Xm.Fix;
	GpsData->Sats = Xm.Sats;
	if (GpsData->Fix > 1) {
		GpsData->Lat = (float)(Xm.Lat / 1e6); // converts from microdegrees to degrees
		GpsData->Lon = (float)(Xm.Lon / 1e6); // converts from microdegrees to degrees
		GpsData->Alt = Xm.CurrentAlt / 100;   // converts centimeters to meters
		GpsData->Hours = (Xm.CurrentTime / 3600) % 24;
		GpsData->Minutes = (Xm.CurrentTime / 60) % 60;
		GpsData->Seconds = Xm.CurrentTime % 60;
		GpsData->Speed = 0;

		if (Xm.Fix == 3 && Xm.CurrentAlt > 0 && Xm.CurrentTime > 0) {
			if (Xm.PreviousTime > 0 && Xm.PreviousAlt > 0) {
				if (timeDifference(Xm.PreviousTime, Xm.CurrentTime) > AscentRateTime) {
					GpsData->AscentRate = calculateAscentRate(Xm.PreviousAlt, Xm.CurrentAlt, Xm.PreviousTime, Xm.CurrentTime);
					Xm.PreviousAlt = Xm.CurrentAlt;
					Xm.PreviousTime = Xm.CurrentTime;
				}
			} else {
				Xm.PreviousAlt = Xm.CurrentAlt;
				Xm.PreviousTime = Xm.CurrentTime;
			}
		}
	}
}

int8_t getFrameStartPosition(const uint8_t* buffer) {
	for (int8_t fpos = 0; fpos <= GPS_FRAME_LEN + XM_PREAMBULE_LEN; fpos++) {
		if (buffer[fpos] == preambule[0]) {
			if (buffer[fpos + 1] == preambule[1] && buffer[fpos + 2] == preambule[2] && buffer[fpos + 3] == preambule[3]) {
				return fpos;
			}
		}
	}
	return -1;
}

void parseXM(GPS* GpsData, const uint8_t* buffer) {
	const int8_t frameStartPosition = getFrameStartPosition(buffer);
	if (frameStartPosition != -1) {
		ParseXmFrame(buffer, frameStartPosition);
		ConvertXmToGpsData(GpsData);
	} else {
		GpsData->Fix = 0;
	}
}
