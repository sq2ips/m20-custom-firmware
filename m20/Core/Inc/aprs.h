#ifndef INC_APRS_H_
#define INC_APRS_H_

#include <stdint.h>

typedef struct TAPRSPacket {
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
} APRSPacket;
#endif