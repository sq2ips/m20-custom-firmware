#ifndef __INC_AFSK_H_
#define __INC_AFSK_H_

#include "main.h"

uint8_t AFSK_is_active();
void AFSK_timer_handler();
void AFSK_start_TX(char* buff, uint8_t len);
void AFSK_stop_TX();

#endif