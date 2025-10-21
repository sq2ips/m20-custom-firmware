#ifndef INC_APRS_H_
#define INC_APRS_H_

#include <stdint.h>

#define APRS_CONTROL_FIELD 0x03
#define APRS_PROTOCOL_ID 0xf0
#define APRS_SPACE_SYMBOL 0x20

//#define APRS_MAX_INFO_LEN 61
#define APRS_MAX_INFO_LEN 100
#define APRS_MAX_PACKET_LEN APRS_MAX_INFO_LEN+32

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

uint8_t encode_APRS_packet(APRSPacket Packet, uint8_t *buff);

#endif