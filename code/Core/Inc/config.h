/*
 * config.h
 *
 *  Created on: Jan 8, 2025
 *      Author: sq2dk
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define PAYLOAD_ID 256	      // sonde payload ID 256 - for 4FSKTEST-V2
#define CALLSIGN "SQ2DK"      // sonde callsign

#define QRG_CW 435100000      // frequency for CW transmissions
#define QRG_FSK4 435100000    // frequency fo horus modulation

#define PA_CW 30              // RF power setting for CW transmission values 0-63
#define PA_FSK4 30            // RF power setting for horus transmission values 0-63

#define CW_DOT_MS 50           // length of dot in ms for CW. Smaller values - faster WPM
#define CW_UNITS_DASH 3        // how many times dash is longer than dot
#define CW_UNITS_GAP 3         // how many times silence between characters is longer than dot

#define FSK4_BAUD 100          // baudrate for horus 4FSK
#define FSK4_SPACE_MULTIPLIER 1  //tone spacing multiplier - 1 for 275Hz, 2 for 550, etc.

#define ADF_CLOCK 9000000      // Hz clock speed of adf7012 chip coming from STM32

#define FSK4_HEADER_LENGTH 8    //length in bytes of 4FSK header

#define RF_BOOST_ACTIVE 0       //RF booster enabled for transmissions about 15dB gain, but more power consumed - normally should be ON(1).

#define ADF_FREQ_CORRECTION 19  // correction of frequency from crystal inaccuracy in 270Hz steps. To be individually set for each sonde.

// GPS configuration
#define RxBuffer_SIZE 32 // one read buffer
#define DataBuffer_SIZE 1024 // one parsing buffer
#define DATA_SIZE 35

#define TIME_DELAY 5000

#endif /* INC_CONFIG_H_ */
