#include "afsk.h"
#include "adf.h"
#include "main.h"
#include "config.h"

#include <stdio.h>

static bool AFSK_Active = false;

bool AFSK_is_active(){
    return AFSK_Active;
}

void AFSK_stop_TX() {
  TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
  TIM2->DIER &= ~(TIM_DIER_UIE); // Disable the interrupt
  
  TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
  adf_RF_off();                  // turn TX off
  AFSK_Active = false;
}

static uint16_t cntr = 0;
static bool current_tone = 0;
void AFSK_timer_handler(){ // Changing tone (TIM2)
    TIM2->CNT = 0;
    if(cntr++ == 1200){
        current_tone = !current_tone;
        printf("current tone %d, cnt %d\r\n", current_tone, TIM21->CNT);
        cntr = 0;
    }

    if(current_tone == 1){
        TIM21->ARR = (uint16_t)((SystemCoreClock / (BELL202_TONE_1 * AFSK_TONE_TIM_PSC)) - 1);
    }else{
        TIM21->ARR = (uint16_t)((SystemCoreClock / (BELL202_TONE_0 * AFSK_TONE_TIM_PSC)) - 1);
    }
    TIM21->CCR1 = (TIM21->ARR+1)/2;
}

void AFSK_start_TX() {
  adf_RF_on(QRG_AFSK, PA_FSK4); // turn on radio TX
  AFSK_Active = true;                                 // change status
  // TIM2 - Modulation, changing tones
  TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable the TIM Counter
  TIM2->PSC = (uint16_t)AFSK_BAUDRATE_TIM_PSC - 1;
  TIM2->ARR = (uint16_t)((SystemCoreClock / (AFSK_BAUDRATE * AFSK_BAUDRATE_TIM_PSC)) - 1); // timer value calculated according to baud rate 999 for 100bd, autoreload 120
                                // for baudrate (auto-reload register)
  TIM2->CR1 |= TIM_CR1_CEN;     // enable timer again
  TIM2->DIER |= TIM_DIER_UIE;   // Enable the interrupt

  // TIM21 - Generating tones
  TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable the TIM Counter
  TIM21->PSC = (uint16_t)AFSK_TONE_TIM_PSC - 1;
  AFSK_timer_handler(); // Handle modulation for setting autoreload for TIM21
                                // for baudrate (auto-reload register)
  TIM21->CR1 |= TIM_CR1_CEN;     // enable timer again
}
