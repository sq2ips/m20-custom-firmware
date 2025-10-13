#include "afsk.h"
#include "adf.h"
#include "main.h"
#include "config.h"

#include <stdio.h>

static bool AFSK_Active = false;
const static uint8_t buff[] = {0x7e, 0x41, 0x05, 0x25, 0x65, 0x02, 0x02, 0x07, 0x39, 0x79, 0x61, 0x41, 0x19, 0x19, 0x47, 0x75, 0x49, 0x11, 0x51, 0x46, 0x02, 0xc7, 0xc0, 0x07, 0x81, 0x06, 0x4e, 0x26, 0x66, 0x16, 0x56, 0x2f, 0x7a, 0x2e, 0x2a, 0x51, 0x22, 0x6e, 0x47, 0x2d, 0x5e, 0x79, 0x59, 0x25, 0x61, 0x7a, 0x41, 0x5e, 0x06, 0x0e, 0x0e, 0x46, 0x66, 0x26, 0x09, 0x53, 0x1b, 0x1b, 0x7b, 0x02, 0x75, 0x7b, 0x27, 0x1b, 0x13, 0x42, 0x22, 0x89, 0x3f, 0x3f};
const static uint32_t buff_len = sizeof(buff);
static uint32_t bit_pos = 0;
static bool current_tone = 0;


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

// 0, N1-1 | N1, N1+N2-1 | N1+N2, N1+N2+buff_len-1 | buff_len, buff_len+N3-1

void AFSK_timer_handler(){ // Changing tone (TIM2)
    TIM2->CNT = 0;

    bool current_bit = 0;
    if(bit_pos < N1_SEGMENT_COUNT*8){ // N1
        current_bit = 0; // N1 is 0x00
    }else if(bit_pos >= N1_SEGMENT_COUNT*8 && bit_pos < (N1_SEGMENT_COUNT+N2_SEGMENT_COUNT)*8){ // N2
        current_bit = (BELL202_N2_N3_FLAG>>(7-(bit_pos%8))) & 1;
    }else if(bit_pos >= (N1_SEGMENT_COUNT+N2_SEGMENT_COUNT)*8 && bit_pos < (N1_SEGMENT_COUNT+N2_SEGMENT_COUNT+buff_len)*8){ // DATA
        current_bit = (buff[(bit_pos/8)-(N1_SEGMENT_COUNT+N2_SEGMENT_COUNT)]>>(7-(bit_pos%8))) & 1;
    }else if(bit_pos >= (N1_SEGMENT_COUNT+N2_SEGMENT_COUNT+buff_len)*8 && bit_pos < (N1_SEGMENT_COUNT+N2_SEGMENT_COUNT+buff_len+N3_SEGMENT_COUNT)*8){ // N3
        current_bit = (BELL202_N2_N3_FLAG>>(7-(bit_pos%8))) & 1;
    }
    bit_pos++;

    if(current_bit == 0){
        TIM21->CNT = 0;
        //TIM21->CNT = TIM21->ARR;
        current_tone = !current_tone; // 0 is encoded by switching from one frequency, to the other (1200hz -> 2200hz or 2200hz -> 1200hz).
    }
    // A 1 is encoded by not changing the frequency.

    if(current_tone == 0){
        TIM21->ARR = (uint16_t)((SystemCoreClock / (BELL202_TONE_A * AFSK_TONE_TIM_PSC)) - 1);
    }else{
        TIM21->ARR = (uint16_t)((SystemCoreClock / (BELL202_TONE_B * AFSK_TONE_TIM_PSC)) - 1);
    }
    TIM21->CCR1 = (TIM21->ARR+1)/2;

    if(bit_pos == (N1_SEGMENT_COUNT+N2_SEGMENT_COUNT+buff_len+N3_SEGMENT_COUNT)*8) AFSK_stop_TX();
}

void AFSK_start_TX() {
  adf_RF_on(QRG_AFSK, PA_FSK4); // turn on radio TX
  AFSK_Active = true;                                 // change status
  current_tone = 0;
  bit_pos = 0;

  // TIM2 - Modulation, changing tones
  TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable the TIM Counter
  TIM2->PSC = (uint16_t)AFSK_BAUDRATE_TIM_PSC - 1;
  TIM2->ARR = (uint16_t)((SystemCoreClock / (AFSK_BAUDRATE * AFSK_BAUDRATE_TIM_PSC)) - 1); // timer value calculated according to baud rate 999 for 100bd, autoreload 120
                                // for baudrate (auto-reload register)
  TIM2->CR1 |= TIM_CR1_CEN;     // enable timer again
  TIM2->DIER |= TIM_DIER_UIE;   // Enable the interrupt

  // TIM21 - Generating tones
  TIM21->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable the TIM Counter
  TIM21->PSC = (uint16_t)AFSK_TONE_TIM_PSC - 1; // set prescaler
  AFSK_timer_handler(); // Handle modulation for setting autoreload for TIM21
                                // for baudrate (auto-reload register)
  TIM21->CCER = LL_TIM_CHANNEL_CH1; // Set PWM channel
  TIM21->CR1 |= TIM_CR1_CEN;     // enable timer again
}
