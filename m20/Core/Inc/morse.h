#ifndef __INC_CW_H_
#define __INC_CW_H_

#include "main.h"

uint8_t CW_is_active();
void CW_start_TX(char *buff, uint8_t lenn);
void CW_stop_TX();
void CW_timer_handler();
void get_time_table(uint8_t letter);

uint8_t get_mh(float lat, float lon, uint8_t size, char *buffer);

#endif