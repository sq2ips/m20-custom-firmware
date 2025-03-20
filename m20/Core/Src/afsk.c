#include "afsk.h"
#include "adf.h"
#include "config.h"

uint16_t AFSK_timer2ReloadValue = 0;

char *AFSK_buffer;
uint8_t AFSK_buffer_len = 0;

uint8_t AFSK_Active = 0;

uint32_t counter = 0;
uint8_t tone_state = 0;

uint8_t AFSK_is_active() {  //returns 1 if transmitter is active for 4FSK

	return AFSK_Active;
}

void AFSK_timer_handler(){
    TIM2->CNT = 0;
    LL_GPIO_TogglePin(ADF_TX_Data_GPIO_Port, ADF_TX_Data_Pin);
    if(counter>=1000){
        counter = 0;
        if(tone_state == 0){
            tone_state = 1;
            AFSK_timer2ReloadValue = (50000 / AFSK_TONE_A) - 1;
            TIM2->ARR = AFSK_timer2ReloadValue;
        }else{
            tone_state = 0;
            AFSK_timer2ReloadValue = (50000 / AFSK_TONE_B) - 1;
            TIM2->ARR = AFSK_timer2ReloadValue;
        }
    }

    counter++;
}

void AFSK_start_TX(char* buff, uint8_t len) {
    AFSK_buffer = buff;
    AFSK_buffer_len = len;

    adf_RF_on(QRG_AFSK, PA_AFSK);                                   //turn on radio TX
    AFSK_Active = 1;                                                    //change status
    TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));                 // Disable the TIM Counter
    AFSK_timer2ReloadValue = (50000 / AFSK_TONE_A) - 1;              //timer value calculated according to baud rate 999 for 100bd
    TIM2->ARR = AFSK_timer2ReloadValue;                           //set timer counter max value to pre-set value for baudrate (auto-reload register)
      TIM2->CR1 |= TIM_CR1_CEN;                               //enable timer again
    TIM2->DIER |= TIM_DIER_UIE;                             //Enable the interrupt
    AFSK_timer_handler();                           //force execution of procedure responsible for interrupt handling
}

void AFSK_stop_TX() {
    TIM2->DIER &= ~(TIM_DIER_UIE); /* Disable the interrupt */
    adf_RF_off();   //turn TX off
    AFSK_Active = 0;
}