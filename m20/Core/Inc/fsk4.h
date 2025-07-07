#ifndef INC_FSK4_H_
#define INC_FSK4_H_

#include <stdint.h>

void FSK4_stop_TX();
bool FSK4_is_active();
void FSK4_timer_handler();
void FSK4_start_TX(char* buff, uint8_t len);

#endif /* INC_HORUS_TRANSMIT_H_ */
