/*   MAIN CONFIG FILE FOR THIS PROJECT   */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/*-----------------------------------------------------------------*/
//  Sonde configuration, parameters that should be changed
#define DEBUG 0

#define TIME_PERIOD 12           // Time betwen starts of transmissions (in seconds) (must be more than 3)

// Horus Binary V2 4FSK mode
#define HORUS_ENABLE 1

const static float QRG_FSK4[] = {435100000}; // Transmitted frequencies array, switched in a loop, add new frequencies (in Hz) after a comma in braces.
#define FSK4_POWER 10              // RF power setting for horus transmission values 0-63

#define HORUS_PAYLOAD_ID 256          // Sonde payload ID 256 - for 4FSKTEST-V2

#define FSK4_BAUD 100           // Baudrate for horus 4FSK
#define FSK4_SPACE_MULTIPLIER 1 // Tone spacing multiplier - 1 for 244Hz, 2 for 488, etc.
#define FSK4_HEADER_LENGTH 8    // Length in bytes of 4FSK header

// APRS (AX.25 AFSK HDLC Bell 202)
#define APRS_ENABLE 1

const static float QRG_AFSK[] = {435100000};
#define AFSK_POWER 10              // RF power setting for horus transmission values 0-63

#define APRS_CALLSING "NO0CALL"
#define APRS_SSID 11

#define APRS_DESTINATION "APZM20"
#define APRS_DESTINATION_SSID 0

#define APRS_PATH_1 "WIDE1"
#define APRS_PATH_1_SSID 1

#define APRS_PATH_2 "WIDE2"
#define APRS_PATH_2_SSID 1

#define APRS_SYMBOL "/O"        // baloon symbol, all symbols: https://www.aprs.org/symbols.html

#define APRS_COMMENT_TELEMETRY 1

#define APRS_COMMENT_TEXT "M20 radiosonde test"

// LED settings
#define LED_MODE 2              // 0 - disabled, 1 - flashes when prepairing tx data before transmit, 2 - GPS fix indication
#define LED_PERIOD 5            // time between LED lighting
#define LED_DISABLE_ALT 1000    // disable led when certain altitude is reached, 0 for always enable

// Radio settings
#define RF_BOOST 0       // RF booster enabled for transmissions about 15dB gain, but more power consumed - normally should be ON(1).

#define ADF_FREQ_CORRECTION 19  // correction of frequency from crystal inaccuracy in 244Hz steps. To be individually set for each sonde.
#define ADF_FSK_DEVIATION 5  // Deviation parameter used in AFSK modem, don't change it without a reason, 5= about 5k5Hz, 10=11kHz
#define ADF_CLOCK 8000000       // Clock speed of adf7012 chip coming from STM32 (in Hz) (set to HSE 8MHz oscilator)

// GPS configuration
#define GPS_TYPE 1              // Type of GPS module: 1 - u-blox | 2 - XM1110

#define GPS_WATCHDOG 1           // Enable GPS watchdog
#define GPS_WATCHDOG_ARC 180    // Time of GPS watchdog after restart counter, how much time (in s) the module has for getting a fix after a restart before next one
//#define GPS_WATCHDOG_MAX_D_TIME 10 // Max GPS time deviation (in s) from last frame
//#define GPS_WATCHDOG_ASCENTRATE 100 // Ascent rate value triggering restart

#define AscentRateTime TIME_PERIOD/2       // Time of ascent rate mesure

#define GPS_DEBUG 0

#define LPS22_ENABLE 1

#define NTC_ENABLE 1

#define BAT_ADC_ENABLE 1

#endif /* INC_CONFIG_H_ */
