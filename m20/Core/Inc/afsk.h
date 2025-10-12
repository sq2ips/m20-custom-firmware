#ifndef INC_AFSK_H_
#define INC_AFSK_H_

#define BELL202_TONE_1 1200
#define BELL202_TONE_0 2200
#define AFSK_TONE_TIM_PSC 1

#define AFSK_BAUDRATE 1
#define AFSK_BAUDRATE_TIM_PSC 100

bool AFSK_is_active();
void AFSK_stop_TX();
void AFSK_timer_handler();
void AFSK_start_TX();

#endif