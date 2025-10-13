#ifndef INC_AFSK_H_
#define INC_AFSK_H_

#define BELL202_TONE_A 1200
#define BELL202_TONE_B 2200
#define AFSK_TONE_TIM_PSC 1

#define AFSK_BAUDRATE 1200
#define AFSK_BAUDRATE_TIM_PSC 100

/*
 *           APRS BELL202/HDLC frame sync segments
 * |-----------|-----------|-----------------|-----------|
 * |    0x00   | 0x7E flag | Data & checksum | 0x7E flag |
 * |-----------|-----------|-----------------|-----------|
 * | N1 octets | N2 octets |                 | N3 octets |
 * |-----------|-----------|-----------------|-----------|
*/

#define BELL202_N2_N3_FLAG 0x7E // sync flag for N2 and N3

#define N1_SEGMENT_COUNT 0
#define N2_SEGMENT_COUNT 100
#define N3_SEGMENT_COUNT 100

bool AFSK_is_active();
void AFSK_stop_TX();
void AFSK_timer_handler();
void AFSK_start_TX();

#endif