/*   MAIN CONFIG FILE FOR THIS PROJECT   */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/*-----------------------------------------------------------------*/
/*----  Sonde configuration, parameters that should be changed ----*/

#define PAYLOAD_ID 256          // Sonde payload ID 256 - for 4FSKTEST-V2

#define CALLSIGN "SQ2IPS"

#define TIME_PERIOD 20           // Time betwen starts of transmissions (in seconds) (must be more than 3)

#define GPS_TYPE 1              // Type of GPS module: 1 - u-blox | 2 - XM1110

#define RF_BOOST_ACTIVE 0       // RF booster enabled for transmissions about 15dB gain, but more power consumed - normally should be ON(1).

/* Horus 4FSK settings */
#define HORUS_EN 1               // Horus 4FSK enable
#define QRG_FSK4 435100000      // Frequency fo horus modulation (in Hz)
#define PA_FSK4 10              // RF power setting for horus transmission values 0-63
/*---------------------*/

#define QRG_AFSK 435300000
#define PA_AFSK 10

#define AFSK_TONE_A 2200
#define AFSK_TONE_B 1200

#define ADF_DEVIATION 5

/* CW morse settings */
#define CW_EN 0                 // Morse enable
#define QRG_CW   435200000      // Frequency fo CW morse modulation (in Hz)
#define PA_CW 10                // RF power setting for CW morse transmission values 0-63

#define MORSE_GRID_SIZE 3       // number of QTH elements to generate 1 for each group for example 3 for AB12CD etc, max is 5

#define MORSE_MS 50             // Length of one morse period in ms
#define MORSE_UNITS_DOT 1
#define MORSE_UNITS_DASH 3
#define MORSE_UNITS_GAP 3
#define MORSE_UNITS_SPACE 7
/*-------------------*/

/* LED settings */
#define LED_MODE 1              // 0 - disabled, 1 - flashes when prepairing tx data before transmit, 2 - GPS fix indication
#define LED_PERIOD 5            // time between LED lighting
#define LED_DISABLE_ALT 1000    // disable led when certain altitude is reached, 0 for always enable
/*--------------*/

#define ADF_FREQ_CORRECTION 19  // correction of frequency from crystal inaccuracy in 270Hz steps. To be individually set for each sonde.

/*-----------------------------------------------------------------*/
/*----- the rest of parameters should not be changed normally -----*/

#define DEBUG // debug information on USART1

/* 4FSK settings */
#define FSK4_BAUD 100           // Baudrate for horus 4FSK
#define FSK4_SPACE_MULTIPLIER 1 // Tone spacing multiplier - 1 for 244Hz, 2 for 488, etc.

#define ADF_CLOCK 8000000       // Clock speed of adf7012 chip coming from STM32 (in Hz) (set to HSE 8MHz oscilator)

#define FSK4_HEADER_LENGTH 8    // Length in bytes of 4FSK header
/*---------------*/

/* Horus encoding config */
#define INTERLEAVER
#define SCRAMBLER
/*-----------------------*/

/* NTC settings */
#define NTC_GPIO_Port NTC_36K_GPIO_Port // resistor select
#define NTC_Pin NTC_36K_Pin
/*--------------*/

/* GPS configuration */
#define AscentRateTime 10       // Time of ascent rate mesure

// type 1
// TODO: check needed max values
#define DATA_SIZE               35 // Max number of NMEA sentences in one parsing
#define SENTENCE_SIZE           82+1 // Max lenght of a NMEA sentence is 82 characters
#define MAX_SENTENCE_ELEMENTS   10 // Max number of NMEA sentence elements (no element with number bigger than 9 is used)
#define SENTENCE_ELEMENT_LEN    12 // Max lenght of a sentence element

// type 2
#define FrameLen 62             // Length of XM1110 frame

#if GPS_TYPE == 1
#define GpsRxBuffer_SIZE 512

#elif GPS_TYPE == 2
#define GpsRxBuffer_SIZE FrameLen * 2
#endif
/*-------------------*/

#endif /* INC_CONFIG_H_ */
