#ifndef INC_APRS_H_
#define INC_APRS_H_

#include <stdint.h>

#define APRS_CONTROL_FIELD 0x03
#define APRS_PROTOCOL_ID 0xf0
#define APRS_SPACE_SYMBOL 0x20

#define APRS_MAX_INFO_LEN 61

typedef struct TAPRSPacket {
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
    float Lat;
    float Lon;
    uint16_t Alt;
    uint8_t Speed;
} APRSPacket;

uint8_t encode_APRS_packet(APRSPacket Packet, uint8_t *buff);

#endif