#ifndef INC_APRS_H_
#define INC_APRS_H_

#include <stdint.h>
#include "config.h"

#define APRS_CONTROL_FIELD 0x03
#define APRS_PROTOCOL_ID 0xf0
#define APRS_SPACE_SYMBOL 0x20

#define APRS_INFO_DATA_LEN 30 // info field data length
#define APRS_MAX_TELEM_LEN 33 // comment telemetry max length
#define APRS_AX25_FORMAT_LEN 32 // AX.25 additional length: 7 bytes * 4 fields (source, destination, path 1, path 2) + 2 control bytes

#ifdef APRS_COMMENT_TEXT
#define APRS_COMMENT_LEN (sizeof(APRS_COMMENT_TEXT)-1) // comment text length (ignore ending '\0')
#endif

#define APRS_MAX_INFO_LEN APRS_INFO_DATA_LEN+APRS_MAX_TELEM_LEN*APRS_COMMENT_TELEMETRY+APRS_COMMENT_LEN*APRS_COMMENT_TEXT_ENABLE

#define APRS_MAX_PACKET_LEN APRS_MAX_INFO_LEN+APRS_AX25_FORMAT_LEN // total packet length

#define KNOTS_TO_KMPH 1.852f // exact
#define FEET_TO_M 0.3048f // exact

typedef struct TAPRSPacket {
    // Base data
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
    float Lat;
    float Lon;
    uint16_t Alt;
    uint8_t Speed;
    // Telemetry in comment
    uint16_t PacketCount;
    uint8_t Sats;
    uint8_t GpsResetCount;
    int8_t Temp;
    int16_t ExtTemp;
    uint16_t Press;
    uint16_t BatVoltage;
} APRSPacket;

uint8_t encode_APRS_packet(APRSPacket Packet, char *buff);

#endif