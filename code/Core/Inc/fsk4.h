/*
 * horus_transmit.h
 *
 *  Created on: Jan 5, 2025
 *      Author: SQ2DK
 */

#ifndef INC_FSK4_H_
#define INC_FSK4_H_

#include "main.h"


void FSK4_stop_TX();
void FSK_4_send_2bit(uint8_t bits_to_send);
uint8_t FSK4_is_active();
void FSK_4_write(char* buff, uint16_t adress_2bit);
void FSK4_timer_handler();
void FSK4_start_TX(char* buff, uint8_t len);


#endif /* INC_HORUS_TRANSMIT_H_ */
