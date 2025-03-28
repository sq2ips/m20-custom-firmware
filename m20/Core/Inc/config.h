/*   MAIN CONFIG FILE FOR THIS PROJECT   */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

//#define DEBUG

/*-----------------------------------------------------------------*/
//  Sonde configuration, parameters that should be changed

#define PAYLOAD_ID 256          // Sonde payload ID 256 - for 4FSKTEST-V2

#define TIME_PERIOD 6           // Time betwen starts of transmissions (in seconds) (must be more than 3)

#define GPS_TYPE 1              // Type of GPS module: 1 - u-blox | 2 - XM1110

#define QRG_FSK4 435100000      // Frequency fo horus modulation (in Hz)

#define PA_FSK4 10              // RF power setting for horus transmission values 0-63
#define RF_BOOST_ACTIVE 1       // RF booster enabled for transmissions about 15dB gain, but more power consumed - normally should be ON(1).

#define ADF_FREQ_CORRECTION 19  // correction of frequency from crystal inaccuracy in 270Hz steps. To be individually set for each sonde.

#define LED_MODE 2              // 0 - disabled, 1 - flashes when prepairing tx data before transmit, 2 - GPS fix indication
#define LED_PERIOD 5            // time between LED lighting
#define LED_DISABLE_ALT 1000    // disable led when certain altitude is reached, 0 for always enable

/*-----------------------------------------------------------------*/
//  the rest of parameters should not be changed normally

#define FSK4_BAUD 100           // Baudrate for horus 4FSK
#define FSK4_SPACE_MULTIPLIER 1 // Tone spacing multiplier - 1 for 244Hz, 2 for 488, etc.

#define ADF_CLOCK 8000000       // Clock speed of adf7012 chip coming from STM32 (in Hz) (set to HSE 8MHz oscilator)

#define FSK4_HEADER_LENGTH 8    // Length in bytes of 4FSK header

/* Horus encoding config */
#define INTERLEAVER
#define SCRAMBLER
/*-----------------------*/

/* GPS configuration */
#define AscentRateTime 10       // Time of ascent rate mesure

// type 1
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
