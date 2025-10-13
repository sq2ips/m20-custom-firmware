#include "afsk.h"
#include "adf.h"
#include "utils.h"
#include "main.h"
#include "config.h"

#include <stdio.h>

static uint16_t sine_table[SINE_TABLE_SIZE];

static bool AFSK_Active = false;

static volatile float phase = 0.0f;
const static float phase_inc_tone_A = (SINE_TABLE_SIZE * BELL202_TONE_A)/(float)AFSK_UPDATE_SAMPLERATE;
const static float phase_inc_tone_B = (SINE_TABLE_SIZE * BELL202_TONE_B)/(float)AFSK_UPDATE_SAMPLERATE;
static float phase_inc;

static uint16_t bit_sample_counter = 0;
static bool current_tone = 0;
static uint8_t bit_pos = 0;

void init_sine_table(void) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        float s = (Sinf(2.0f * PI * i / SINE_TABLE_SIZE) + 1.0f) * 0.5f;
        sine_table[i] = (uint16_t)(s * (AFSK_PWM_TIM_ARR+1)); // +1 ???
    }
}

bool AFSK_is_active(){
    return AFSK_Active;
}

void AFSK_stop_TX() {
  TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
  TIM2->DIER &= ~(TIM_DIER_UIE); // Disable the interrupt
  
  TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable the counter
  adf_RF_off();                  // turn TX off
  AFSK_Active = false; // flag to off
}

void AFSK_timer_handler(){ // Changing tone (TIM2)
    TIM2->CNT = 0;

    uint16_t index = (uint16_t)phase;
    if (index >= SINE_TABLE_SIZE) index -= SINE_TABLE_SIZE;

    TIM21->CCR1 = sine_table[index];

    phase += phase_inc;
    if (phase >= SINE_TABLE_SIZE) phase -= SINE_TABLE_SIZE;

    if (++bit_sample_counter >= AFSK_UPDATE_SAMPLERATE/AFSK_BAUDRATE){
        bit_sample_counter = 0;

        if(((0x7E>>(7-(bit_pos++%8))) & 1) == 0) current_tone = !current_tone;

        if(current_tone==1){
            phase_inc = phase_inc_tone_A;
        }else{
            phase_inc = phase_inc_tone_B;
        }
    }
}

void AFSK_start_TX() {
  init_sine_table();
  adf_RF_on(QRG_AFSK, PA_FSK4); // turn on radio TX
  AFSK_Active = true;                                 // change status
  current_tone = 0;
  phase_inc = phase_inc_tone_A;

  // TIM2 - Modulation, changing duty cycles
  TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable the TIM Counter
  TIM2->PSC = AFSK_UPDATE_TIM_PSC;
  TIM2->ARR = (SystemCoreClock/AFSK_UPDATE_SAMPLERATE)-1;
  TIM2->CR1 |= TIM_CR1_CEN;     // enable timer again
  TIM2->DIER |= TIM_DIER_UIE;   // Enable the interrupt

  // TIM21 - PWM timer, Generating tones
  TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable the TIM Counter
  TIM21->PSC = AFSK_PWM_TIM_PSC;
  TIM21->ARR = AFSK_PWM_TIM_ARR;

  TIM21->CCER |= LL_TIM_CHANNEL_CH1; // Set PWM channel
  TIM21->CR1 |= TIM_CR1_CEN;     // enable timer again

  AFSK_timer_handler();
}
