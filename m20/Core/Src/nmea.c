/*
 *  Based on https://github.com/sztvka/stm32-nmea-gps-hal
 *  Adapted by SQ2IPS
 */

#include "nmea.h"
#include "main.h"
#include "config.h"

#include <stdlib.h>
#include <string.h>

char *data[DATA_SIZE];

uint8_t correct = 0;
uint16_t olddTime = 0;
float olddAlt = 0;

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
    int receivedHash = strtol(recv_crc, NULL, 16);
    if (crc == receivedHash) {
        return 1;
    }else{
        return 0;
    }
}

int getValues(char*inputString, char *values[]){
    char *marker = strtok(inputString, ",");
    int counter = 0;
    while (marker != NULL) {
        values[counter++] = malloc(strlen(marker) + 1); //free later!!!!!!
        strcpy(values[counter - 1], marker);
        marker = strtok(NULL, ",");
    }
    return counter;
}

int nmea_GGA(NMEA *nmea_data, char*inputString){
	char *values[25];
    memset(values, 0, sizeof(values));
	int counter = getValues(inputString, values);

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

            float altitude = strtof(values[9], NULL);

            if(altitude !=0){
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
            
            nmea_data->Sats = strtol(values[7], NULL, 10);

            //nmea_data->Fix = strtol(values[6], NULL, 10);

            float hdop = strtof(values[8], NULL);
            if(nmea_data->Fix > 1) nmea_data->HDOP = hdop!=0 ? hdop : nmea_data->HDOP;

            for(int i=0; i<counter; i++) free(values[i]);
            return 1;
        }

    }

    for(int i=0; i<counter; i++) free(values[i]);
    return 0;
}


int nmea_GSA(NMEA *nmea_data, char*inputString){
	char *values[25];
    memset(values, 0, sizeof(values));
	int counter = getValues(inputString, values);

    nmea_data->Fix = strtol(values[2], NULL, 10);
    int satelliteCount = 0;
    for(int i=3; i<15; i++){
        if(values[i][0] != 0){
            satelliteCount++;
        }
    }
    //nmea_data->Sats = satelliteCount;
    for(int i=0; i<counter; i++) free(values[i]);
    return 1;
}



int nmea_GLL(NMEA *nmea_data, char*inputString) {
	char *values[25];
    memset(values, 0, sizeof(values));
	int counter = getValues(inputString, values);

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
            for(int i = 0; i<counter; i++) free(values[i]);
            return 1;
        }
    }

    for(int i = 0; i<counter; i++) free(values[i]);
    return 0;

}

int nmea_VTG(NMEA *nmea_data, char*inputString) {
	char *values[25];
    memset(values, 0, sizeof(values));
	int counter = getValues(inputString, values);

    float speed = strtof(values[5], NULL); // 5 for kph, 3 for knots
    if(speed != 0.0) nmea_data->Speed = speed;

    for(int i=0; i<counter; i++) free(values[i]);
    return 1;
}

void ParseNMEA(NMEA *nmea_data, uint8_t *buffer){
	memset(data, 0, sizeof(data));
	char * token = strtok(buffer, "$");
	uint8_t cnt = 0;
	while( token != NULL && cnt < DATA_SIZE) {
		data[cnt] = malloc(strlen(token)+1);
		strcpy(data[cnt], token);
		cnt++;

		token = strtok(NULL, "$");
	}
	correct = 0;
    for(uint8_t i = 0; i<cnt; i++){
    	if(strstr(data[i], "GN")!=NULL && strstr(data[i], "\r\n")!=NULL && checksum(data[i])){
    		if(strstr(data[i], "GNGLL")!=NULL){
    			nmea_GLL(nmea_data, data[i]);
        		correct++;
    	    }else if(strstr(data[i], "GNGSA")!=NULL){
    	    	nmea_GSA(nmea_data, data[i]);
        		correct++;
    	    }else if(strstr(data[i], "GNGGA")!=NULL){
    	    	nmea_GGA(nmea_data, data[i]);
        		correct++;
    	    }else if(strstr(data[i], "GNVTG")!=NULL){
    	    	nmea_VTG(nmea_data, data[i]);
        		correct++;
    	    }
    	}
    	nmea_data->Corr = correct;
        if(correct == 0){
            nmea_data->Fix = 0;
        }
    }
    //printf("\r\n%d\r\n", SysTick_counter);
    for(uint8_t i = 0; i<cnt; i++) free(data[i]);
}