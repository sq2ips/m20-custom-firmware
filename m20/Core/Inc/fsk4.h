#ifndef INC_FSK4_H_
#define INC_FSK4_H_

#include <stdint.h>

#define FSK4_TIM_PSC 119

extern bool FSK4_Active;

void FSK4_stop_TX();
void FSK4_timer_handler();
void FSK4_start_TX(char* buff, uint8_t len);

#endif /* INC_FSK4_H_ */
