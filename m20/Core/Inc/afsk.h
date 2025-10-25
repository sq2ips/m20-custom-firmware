#ifndef INC_AFSK_H_
#define INC_AFSK_H_

#include <stdint.h>

#define MODEM_CLOCK_RATE 12000000 // System clock

#define BELL202_MARK 1200UL // Mark tone 1200Hz tuned
#define BELL202_SPACE 2200UL // Space tone 2200Hz tuned
#define AFSK_PWM_TIM_PSC 0 // Prescaler 1
#define AFSK_PWM_TIM_ARR 255 // Must be max value from sine lookup table. Generates PWM frequency and sampling interrupt (therefore it's also sample rate): 12MHz / (255+1) Hz = 46.875kHz

#define AFSK_BAUDRATE 1200UL // Baudrate (and bitrate)

/*
 *           APRS BELL202/HDLC frame sync segments
 * |-----------|-----------|-----------------|-----------|
 * |    0x00   | 0x7E flag | Data & checksum | 0x7E flag |
 * |-----------|-----------|-----------------|-----------|
 * | N1 octets | N2 octets |                 | N3 octets |
 * |-----------|-----------|-----------------|-----------|
*/

#define AFSK_SYNC_FLAG 0b01111110 // 0x7E sync flag for N2 and N3

#define N1_SYNC_COUNT 20 // number of N1 octets
#define N2_SYNC_COUNT 10 // number of N2 octets
#define N3_SYNC_COUNT 10 // number of N3 octets

void AFSK_stop_TX();
void AFSK_start_TX(char *buffer, uint16_t buffer_len);

extern bool AFSK_Active;
void AFSK_timer_handler();

#endif