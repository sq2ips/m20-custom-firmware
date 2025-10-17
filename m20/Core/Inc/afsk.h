#ifndef INC_AFSK_H_
#define INC_AFSK_H_

#define BELL202_MARK 1200 // Mark tone 1200Hz tuned
#define BELL202_SPACE 2200 // Space tone 2200Hz tuned
#define AFSK_PWM_TIM_PSC 0 // Prescaler 1
#define AFSK_PWM_TIM_ARR 249 // PWM frequency: 12MHz / (249+1) Hz = 48kHz

#define AFSK_BAUDRATE 1200 // Baudrate (and bitrate)
#define AFSK_UPDATE_TIM_PSC 0 // Prescaler 1
#define AFSK_UPDATE_SAMPLERATE 12000 // update timer frequency (at with the duty cycle will be updated): 12MHz / 12kHz = 1000-1 ARR value

#define SINE_TABLE_SIZE 512 // Size of sine table

/*
 *           APRS BELL202/HDLC frame sync segments
 * |-----------|-----------|-----------------|-----------|
 * |    0x00   | 0x7E flag | Data & checksum | 0x7E flag |
 * |-----------|-----------|-----------------|-----------|
 * | N1 octets | N2 octets |                 | N3 octets |
 * |-----------|-----------|-----------------|-----------|
*/

#define BELL202_N2_N3_FLAG 0b01111110 // 0x7E sync flag for N2 and N3

#define N2_SEGMENT_COUNT 100 // number of N2 octets
#define N3_SEGMENT_COUNT 10 // number of N3 octets

bool AFSK_is_active();
void AFSK_stop_TX();
void AFSK_timer_handler();
void AFSK_start_TX();

#endif