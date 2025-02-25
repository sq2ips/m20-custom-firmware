/*
 *  Based on https://github.com/sztvka/stm32-nmea-gps-hal
 *  Adapted by SQ2IPS
 */

#include "nmea.h"
#include "main.h"
#include "config.h"

#ifdef DEBUG
#include <stdio.h>
#endif
#include <string.h>

char data[DATA_SIZE][SENTENCE_SIZE];

uint8_t correct = 0;
uint16_t olddTime = 0;
uint16_t olddAlt = 0;

uint16_t a_strtof(char *buffer){
    uint8_t d_pos = 0;
    uint16_t value = 0;

    while(buffer[d_pos]!='.'){
        if(buffer[d_pos] == '\0') return 0;
        d_pos++;
    }
    uint16_t e = 1;
    for(int8_t pos = d_pos-1; pos >= 0; pos--){
        value += (buffer[pos]-'0') * e;
        e *= 10;
    }
    if((buffer[d_pos+1]-'0') >= 5) value++; // rounding first decimal place

    return value;
}

uint8_t checksum(char *nmea_frame)
{
    //if you point a string with less than 5 characters the function will read outside of scope and crash the mcu.
    if(strlen(nmea_frame) < 5) return 0;
    char recv_crc[2];
    recv_crc[0] = nmea_frame[strlen(nmea_frame) - 4];
    recv_crc[1] = nmea_frame[strlen(nmea_frame) - 3];
    int crc = 0;
    int i;

    //exclude the CRLF plus CRC with an * from the end
    for (i = 0; i < strlen(nmea_frame) - 5; i ++) {
        crc ^= nmea_frame[i];
    }
    int receivedHash = 0;
    for (int i = 0; i < 2; i++) {
        receivedHash <<= 4;  // Shift left by 4 bits (equivalent to multiplying by 16)
    
        if (recv_crc[i] >= '0' && recv_crc[i] <= '9') {
            receivedHash += recv_crc[i] - '0';  // Convert '0'-'9' to 0-9
        } else if (recv_crc[i] >= 'A' && recv_crc[i] <= 'F') {
            receivedHash += recv_crc[i] - 'A' + 10;  // Convert 'A'-'F' to 10-15
        } else {
            return 0;
        }
    }

    if (crc == receivedHash) {
        return 1;
    }else{
        return 0;
    }
}

uint8_t getValues(char *inputString, char values[MAX_SENTENCE_ELEMENTS][SENTENCE_ELEMENT_LEN]) {
    uint8_t pos = 0;
    uint8_t d_pos = 0;
    uint8_t cnt = 0;
    char buffer[SENTENCE_ELEMENT_LEN];
    memset(buffer, 0, SENTENCE_ELEMENT_LEN);

    while (pos < strlen(inputString) && inputString[pos] != '\n' && pos < SENTENCE_SIZE && cnt < MAX_SENTENCE_ELEMENTS) {
        if (inputString[pos] == ',') {
            // If the length of the value is within buffer limits
            if (pos - d_pos < SENTENCE_ELEMENT_LEN) {
                strncpy(values[cnt], buffer, pos - d_pos);  // Copy the buffer into values[cnt]
                memset(values[cnt] + (pos - d_pos), 0, SENTENCE_ELEMENT_LEN - (pos - d_pos));  // Ensure null termination
                //cnt++;
            }
            cnt++;
            // Reset buffer and update d_pos to start from next character
            memset(buffer, 0, SENTENCE_ELEMENT_LEN);
            d_pos = pos + 1;
        } else {
            // Check if writing to buffer exceeds its size
            if (pos - d_pos < SENTENCE_ELEMENT_LEN - 1) {  // Ensure space for null terminator
                buffer[pos - d_pos] = inputString[pos];
            }
        }
        pos++;
    }
    return cnt;
}

int nmea_GGA(NMEA *nmea_data, char *inputString){
	char values[MAX_SENTENCE_ELEMENTS][SENTENCE_ELEMENT_LEN];
    memset(values, 0, sizeof(values));
	uint8_t len = getValues(inputString, values);
    if(len<9) return 0;

    uint8_t h = (values[1][0]-'0')*10 + (values[1][1]-'0');
    uint8_t m = (values[1][2]-'0')*10 + (values[1][3]-'0');
    uint8_t s = (values[1][4]-'0')*10 + (values[1][5]-'0');

    if(h<24 && m<=60 && s<=60){
        nmea_data->Hours = h;
        nmea_data->Minutes = m;
        nmea_data->Seconds = s;
    }
    
    uint8_t lonSide = values[5][0];
    uint8_t latSide = values[3][0];
    if(latSide == 'S' || latSide == 'N'){
        int deg = (values[2][0] - '0') * 10 + (values[2][1] - '0');
        double min = (values[2][2] - '0') * 10 + (values[2][3] - '0');
        double min_m;
        for(int i = 0; i<5; i++){
            min_m = values[2][i+5] - '0';
            for(int j =0; j<=i; j++) min_m /= 10;
            min += min_m;
        }
        float lat = deg + (min/60.0);

        deg = (values[4][0] - '0') * 100 + (values[4][1] - '0') * 10 + (values[4][2] - '0');
        min = (values[4][3] - '0') * 10 + (values[4][4] - '0');
        for(int i = 0; i<5; i++){
            min_m = values[4][i+6] - '0';
            for(int j = 0; j<=i; j++) min_m /= 10;
            min += min_m;
        }
        float lon = deg + (min/60.0);

        if(lat<=90.0 && lon<=180.0){
            nmea_data->Lat = lat;
            nmea_data->Lon = lon;
            if(latSide == 'S') nmea_data->Lat *= -1;
            if(lonSide == 'W') nmea_data->Lon *= -1;


            uint16_t altitude = a_strtof(values[9]);

            if(altitude != 0){
                nmea_data->Alt = altitude;
                uint16_t currentTime = h*3600+m*60+s;
                if(olddTime==0 || currentTime == 0){
                    olddAlt = nmea_data->Alt;
                    olddTime = currentTime;
                }
                if((currentTime-olddTime)>=AscentRateTime){
                    nmea_data->AscentRate = (nmea_data->Alt-olddAlt)/(currentTime-olddTime);
                    olddAlt = nmea_data->Alt;
                    olddTime = currentTime;
                }
            }
            
            nmea_data->Sats = (values[7][0]-'0')*10 + (values[7][1]-'0');

            //nmea_data->Fix = values[6][0]-'0';

            //float hdop = strtof(values[8], NULL);
            //if(nmea_data->Fix > 1) nmea_data->HDOP = hdop!=0 ? hdop : nmea_data->HDOP;

            return 1;
        }

    }

    return 0;
}


int nmea_GSA(NMEA *nmea_data, char*inputString){
	char values[MAX_SENTENCE_ELEMENTS][SENTENCE_ELEMENT_LEN];
    memset(values, 0, sizeof(values));
	uint8_t len = getValues(inputString, values);
    if(len<2) return 0;

    uint8_t fix = (values[2][0]-'0');
    if(fix<=3){
        nmea_data->Fix = fix;
    }else{
        nmea_data->Fix = 0;
        return 0;
    }

    /*int satelliteCount = 0;
    for(int i=3; i<15; i++){
        if(values[i][0] != 0){
            satelliteCount++;
        }
    }
    nmea_data->Sats = satelliteCount;*/
    return 1;
}



int nmea_GLL(NMEA *nmea_data, char*inputString) {
	char values[MAX_SENTENCE_ELEMENTS][SENTENCE_ELEMENT_LEN];
    memset(values, 0, sizeof(values));
	uint8_t len = getValues(inputString, values);
    if(len<3) return 0;

    uint8_t lonSide = values[4][0];
    uint8_t latSide = values[2][0];
    if(latSide == 'S' || latSide == 'N'){
        int deg = (values[1][0] - '0') * 10 + (values[1][1] - '0');
        double min = (values[1][2] - '0') * 10 + (values[1][3] - '0');
        double min_m;
        for(int i = 0; i<5; i++){
            min_m = values[1][i+5] - '0';
            for(int j =0; j<=i; j++) min_m /= 10;
            min += min_m;
        }
        float lat = deg + (min/60.0);

        deg = (values[3][0] - '0') * 100 + (values[3][1] - '0') * 10 + (values[3][2] - '0');
        min = (values[3][3] - '0') * 10 + (values[3][4] - '0');
        for(int i = 0; i<5; i++){
            min_m = values[3][i+6] - '0';
            for(int j = 0; j<=i; j++) min_m /= 10;
            min += min_m;
        }
        float lon = deg + (min/60.0);

        if(lat<=90.0 && lon<=180.0){
            nmea_data->Lat = lat;
            nmea_data->Lon = lon;
            if(latSide == 'S') nmea_data->Lat *= -1;
            if(lonSide == 'W') nmea_data->Lon *= -1;
            return 1;
        }
    }

    return 0;

}

int nmea_VTG(NMEA *nmea_data, char*inputString) {
	char values[MAX_SENTENCE_ELEMENTS][SENTENCE_ELEMENT_LEN];
    memset(values, 0, sizeof(values));
	uint8_t len = getValues(inputString, values);
    if(len<5) return 0;

    nmea_data->Speed = a_strtof(values[5]); // 5 for kph, 3 for knots 
    return 1;
}

void ParseNMEA(NMEA *nmea_data, uint8_t *buffer){
	memset(data, 0, sizeof(data));
	char * token = strtok((char *)buffer, "$");
	uint8_t cnt = 0;
	while( token != NULL && cnt < DATA_SIZE) {
        if(strlen(token)<SENTENCE_SIZE){
            strncpy(data[cnt], token, SENTENCE_SIZE - 1);
		    data[cnt][SENTENCE_SIZE - 1] = '\0';
            cnt++;
        }
		token = strtok(NULL, "$");
	}
	correct = 0;
    for(uint8_t i = 0; i<cnt; i++){
    	if(strstr(data[i], "GN")!=NULL && strstr(data[i], "\r\n")!=NULL && checksum(data[i])){
            #ifdef DEBUG
		    //printf(">%s", data[i]);
    		#endif
            if(strstr(data[i], "GNGLL")!=NULL){
    			if(nmea_GLL(nmea_data, data[i])) correct++;
    	    }else if(strstr(data[i], "GNGSA")!=NULL){
    	    	if(nmea_GSA(nmea_data, data[i])) correct++;
    	    }else if(strstr(data[i], "GNGGA")!=NULL){
    	    	if (nmea_GGA(nmea_data, data[i])) correct++;
    	    }else if(strstr(data[i], "GNVTG")!=NULL){
    	    	if (nmea_VTG(nmea_data, data[i])) correct++;
    	    }
    	}
    	nmea_data->Corr = correct;
        if(correct == 0){
            nmea_data->Fix = 0;
        }
    }
}