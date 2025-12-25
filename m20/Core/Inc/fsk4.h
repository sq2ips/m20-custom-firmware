#ifndef INC_FSK4_H_
#define INC_FSK4_H_

#include <stdint.h>

#define FSK4_TIM_PSC 119

#define FSK4_BAUD 100           // Baudrate for horus 4FSK
#define FSK4_SPACE_MULTIPLIER 1 // Tone spacing multiplier - 1 for 244Hz, 2 for 488, etc.
#define FSK4_HEADER_LENGTH 8    // Length in bytes of 4FSK header

extern bool FSK4_Active;

void FSK4_stop_TX();
void FSK4_timer_handler();
void FSK4_start_TX(char* buff, uint8_t len);

#endif /* INC_FSK4_H_ */
