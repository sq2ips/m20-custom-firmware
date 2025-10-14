#ifndef INC_AFSK_H_
#define INC_AFSK_H_

#define BELL202_MARK 1300
#define BELL202_SPACE 2350
#define AFSK_PWM_TIM_PSC 0
#define AFSK_PWM_TIM_ARR 119 // PWM frequency: 12MHz / (249+1) Hz = 48kHz

#define AFSK_BAUDRATE 1200
#define AFSK_UPDATE_TIM_PSC 0
#define AFSK_UPDATE_SAMPLERATE 12000 // update timer frequency (at with the duty cycle will be updated): 12MHz / 12kHz = 1000-1 ARR value

#define SINE_TABLE_SIZE 512

/*
 *           APRS BELL202/HDLC frame sync segments
 * |-----------|-----------|-----------------|-----------|
 * |    0x00   | 0x7E flag | Data & checksum | 0x7E flag |
 * |-----------|-----------|-----------------|-----------|
 * | N1 octets | N2 octets |                 | N3 octets |
 * |-----------|-----------|-----------------|-----------|
*/

#define BELL202_N2_N3_FLAG 0b01111110 // sync flag for N2 and N3

#define N2_SEGMENT_COUNT 100
#define N3_SEGMENT_COUNT 10

bool AFSK_is_active();
void AFSK_stop_TX();
void AFSK_timer_handler();
void AFSK_start_TX();

#endif