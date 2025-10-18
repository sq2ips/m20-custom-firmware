#ifndef INC_APRS_H_
#define INC_APRS_H_

#include <stdint.h>

#define APRS_CONTROL_FIELD 0x03
#define APRS_PROTOCOL_ID 0xf0

typedef struct TAPRSPacket {
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
} APRSPacket;

uint16_t encode_APRS_packet(APRSPacket Packet, uint8_t *buff);

#endif