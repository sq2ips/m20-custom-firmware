#include "morse.h"
#include "config.h"
#include "main.h"
#include "adf.h"

#include <stdint.h>
#include <math.h>

//morse code table. table - 0 to Z as per ascii - dec 48
//first digit - dash"-" coded as 1, dot "." coded as 0, unused bits as 0
//nd digit - number of dash-dots in symbol

const uint8_t morse_code[][2] = {
    { 0b11111000, 5},   //0
    { 0b01111000, 5},   //1
    { 0b00111000, 5},   //2
    { 0b00011000, 5},   //3
    { 0b00001000, 5},   //4
    { 0b00000000, 5},   //5
    { 0b10000000, 5},   //6
    { 0b11000000, 5},   //7
    { 0b11100000, 5},   //8
    { 0b11110000, 5},   //9
    { 0b01000000, 2},   //A
    { 0b10000000, 4},   //B
    { 0b10100000, 4},   //C
    { 0b10000000, 3},   //D
    { 0b00000000, 1},   //E
    { 0b00100000, 4},   //F
    { 0b11000000, 3},   //G
    { 0b00000000, 4},   //H
    { 0b00000000, 2},   //I
    { 0b01110000, 4},   //J
    { 0b10100000, 3},   //K
    { 0b01000000, 4},   //L
    { 0b11000000, 2},   //M
    { 0b10000000, 2},   //N
    { 0b11100000, 3},   //O
    { 0b01100000, 4},   //P
    { 0b11010000, 4},   //Q
    { 0b01000000, 3},   //R
    { 0b00000000, 3},   //S
    { 0b10000000, 1},   //T
    { 0b00100000, 3},   //U
    { 0b00010000, 4},   //V
    { 0b01100000, 3},   //W
    { 0b10010000, 4},   //X
    { 0b10110000, 4},   //Y
    { 0b11000000, 4},   //Z
    { 0b10101101, 8},   // [ - but it is CQ
    { 0b10000000, 4}    // | - but it is DE
};

uint8_t CW_active = 0;
char *CW_buffer;
uint8_t CW_buffer_len = 0;

uint8_t CW_buffer_pos = 0;
uint8_t wait_counter = 0;

uint8_t time_table[20]; // TODO check max len
uint8_t time_table_len = 0;
uint8_t time_table_pos = 0;

uint16_t CW_timer2ReloadValueMultip;

uint8_t CW_is_active(){
	return CW_active;
}

uint8_t get_symbol_code(uint8_t letter){
	uint8_t symbol_code = 255;
	uint8_t symbol = letter;

    if (symbol == 32) return symbol;

	if ((symbol > 96 ) && (symbol < 123))   //we have lower case
		    { symbol_code= symbol - 32;}    //make it into upper case

	if ((symbol > 47)  && (symbol < 59))  // digits
	    { symbol_code = symbol-48;}
	if ((symbol > 64 ) && (symbol < 93))  // letters
	    { symbol_code = symbol - 55;}
    
    return symbol_code;
}

void get_time_table(uint8_t letter){
    uint8_t symbol_code = get_symbol_code(letter);
    uint8_t symbol_len = morse_code[symbol_code][1];
    uint8_t symbol = morse_code[symbol_code][0];
    uint8_t pos = 0;
    if(symbol_code == 32){
        time_table[pos] = MORSE_UNITS_SPACE;
        pos++;
    }else{
        for(uint8_t cnt = 0; cnt<symbol_len; cnt++){
            uint8_t dot_dash = (symbol << cnt) & 128;
            if(dot_dash == 128){ // dash
                time_table[pos] = MORSE_UNITS_DASH | 128;
            }else{ // dot
                time_table[pos] = MORSE_UNITS_DOT | 128;
            }
            pos++;
            time_table[pos] = MORSE_UNITS_DOT;
            pos++;
        }
        time_table[pos-1] = MORSE_UNITS_GAP;
    }
    time_table_len = pos;
}

void CW_timer_handler(){
    TIM2->CNT = 0;
    adf_RF_off();
    if(time_table_pos == time_table_len){
        time_table_pos = 0;
        if(time_table_len != 0){
            CW_buffer_pos++;
        }
        if(CW_buffer_pos == CW_buffer_len){
            CW_active = 0;
        }else{
            get_time_table(CW_buffer[CW_buffer_pos]);
        }
    }
    if(CW_active == 1){
        if((time_table[time_table_pos] & 128) == 128){
            adf_RF_on(QRG_CW, PA_CW);
        }
        TIM2->ARR = CW_timer2ReloadValueMultip * (time_table[time_table_pos] & 127); // change timer reload value according to wait time
        time_table_pos++;
    }
}

void CW_start_TX(char *buff, uint8_t lenn){
    CW_buffer = buff;
    CW_buffer_len = lenn;

    time_table_len = 0;
    CW_buffer_pos = 0;
    time_table_pos = 0;

    wait_counter = 0;

    CW_active = 1;
    TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));                 // Disable the TIM Counter
    CW_timer2ReloadValueMultip = (MORSE_MS * 100) - 1;              // initial timer value calculated according to MORSE_MS
    TIM2->ARR = CW_timer2ReloadValueMultip;                           //set timer counter max value to pre-set value for baudrate (auto-reload register)
    TIM2->CR1 |= TIM_CR1_CEN;                               //enable timer again
    TIM2->DIER |= TIM_DIER_UIE;                             //Enable the interrupt
    CW_timer_handler();
}

void CW_stop_TX(){
    TIM2->DIER &= ~(TIM_DIER_UIE); /* Disable the interrupt */
    adf_RF_off();   // ensure off
    CW_active = 0;
}

// based on https://github.com/sp6q/maidenhead/
uint8_t get_mh(float lat, float lon, uint8_t size, char *locator) {
    float LON_F[]={20,2.0,0.083333,0.008333,0.00034720833333};
    float LAT_F[]={10,1.0,0.0416665,0.004166,0.00017358333333};
    uint8_t i;
    lon += 180;
    lat += 90;

    if (size <= 0 || size > 5) size = 3;

    for (i = 0; i < size; i++){
        if (i % 2 == 1) {
            locator[i*2] = (char) (lon/LON_F[i] + '0');
            locator[i*2+1] = (char) (lat/LAT_F[i] + '0');
        } else {
            locator[i*2] = (char) ((uint8_t) (lon/LON_F[i]) + 65);
            locator[i*2+1] = (char) ((uint8_t) (lat/LAT_F[i] + 65));
        }
        lon = fmod(lon, LON_F[i]);
        lat = fmod(lat, LAT_F[i]);
    }
    locator[i*2]=0;
    return size * 2;
}