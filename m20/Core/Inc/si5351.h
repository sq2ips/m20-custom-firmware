#ifndef INC_SI5351_H_
#define INC_SI5351_H_

#include "stdint.h"

#define SI5351_ADDR 0x60

void si5351_write(uint8_t reg, uint8_t val);

#endif