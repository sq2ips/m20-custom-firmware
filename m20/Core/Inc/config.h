/*   MAIN CONFIG FILE FOR THIS PROJECT   */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/*-----------------------------------------------------------------*/
//  Sonde configuration, parameters that should be changed

#define TIME_PERIOD 12 // Time betwen starts of transmissions (in seconds) (must be more than 3)

// Horus Binary V2 4FSK mode
#define HORUS_ENABLE 1

const static float QRG_FSK4[] = {435100000};       // 4FSK transmitted frequencies array, switched in a loop, add new frequencies (in Hz) after a comma in braces. Commonly used
                      // frequencies: https://github.com/projecthorus/horusdemodlib/wiki#commonly-used-frequencies
#define FSK4_POWER 10 // RF power setting for horus transmission values 0-63

#define HORUS_PAYLOAD_ID                                                                                                                             \
	256 // Sonde payload ID 256 - for 4FSKTEST-V2, change this for real flight, refer to
	    // https://github.com/projecthorus/horusdemodlib/wiki#how-do-i-transmit-it

// APRS (AX.25 AFSK HDLC Bell 202)
#define APRS_ENABLE 1

#define TX_PAUSE 1000 // Delay between HORUS and APRS

const static float QRG_AFSK[] = {435100000};       // AFSK transmitted frequencies array, switched in a loop, add new frequencies (in Hz) after a comma in braces.
#define AFSK_POWER 10 // RF power setting AFSK transmission values 0-63

#define APRS_CALLSIGN "NOCALL" // Sonde callsign, max 6 digits, change this for your callsign for real flight
#define APRS_SSID 11           // Sonde SSID, 11 is "balloons, aircraft, spacecraft, etc", refer to https://www.aprs.org/aprs11/SSIDs.txt

#define APRS_DESTINATION "APRM20" // Destination adress, characterizing a M20 transmitter, max 6 digits
#define APRS_DESTINATION_SSID 0   // Default 0 SSID

#define APRS_PATH "WIDE2" // Path, max 6 digits according to https://www.aprs.org/balloons.html
#define APRS_PATH_SSID 1  // Path SSID

#define APRS_SYMBOL "/O" // baloon symbol, all symbols: https://www.aprs.org/symbols.html, needs to be /O for showing on Sondehub

#define APRS_COMMENT_TELEMETRY 1 // Telemetry in coment field

#define APRS_COMMENT_TEXT_ENABLE 1
#define APRS_COMMENT_TEXT "M20 radiosonde test" // Additional text in comment field

// LED settings
#define LED_MODE 2                 // 0 - disabled, 1 - flashes when prepairing tx data before transmit, 2 - GPS fix indication
#define LED_PERIOD 5               // time between LED lighting
#define LED_DISABLE_ALT 1000       // disable led when certain altitude is reached, 0 for always enable
#define LED_MODE_2_BLINK_TIME 100  // single LED blink time in GPS fix mode
#define LED_MODE_2_BLINK_PAUSE 200 // pause time between LED blinks in GPS fix mode

// Radio settings
#define RF_BOOST 1 // RF booster enabled for transmissions about 15dB gain, but more power consumed - normally should be ON(1).

#define ADF_FREQ_CORRECTION 19 // correction of frequency from crystal inaccuracy in 244Hz steps. To be individually set for each sonde.
#define ADF_FSK_DEVIATION 5    // Deviation parameter used in AFSK modem, don't change it without a reason, 5= about 5k5Hz, 10=11kHz

// GPS configuration
#define GPS_TYPE 1 // Type of GPS module: 1 - u-blox | 2 - XM1110

#define GPS_WATCHDOG 1 // Enable GPS watchdog
#define GPS_WATCHDOG_ARC                                                                                                                             \
	180 // Time of GPS watchdog after restart counter, how much time (in s) the module has for getting a fix after a restart before next one
// #define GPS_WATCHDOG_MAX_D_TIME 10 // Max GPS time deviation (in s) from last frame
// #define GPS_WATCHDOG_ASCENTRATE 100 // Ascent rate value triggering restart

#define AscentRateTime TIME_PERIOD / 2 // Time of ascent rate mesure

#define LPS22_ENABLE 1

#define NTC_ENABLE 1

#define BAT_ADC_ENABLE 1

// Debug
#define DEBUG 0

#define GPS_DEBUG 0

#define GPS_RAW_DEBUG 0

#endif /* INC_CONFIG_H_ */
