
/*
 * nmea.c
 *
 *  Created on: Dec 30, 2024
 *      Author: pawel
 */
#include "nmea.h"

char *data[DATA_SIZE];

uint8_t correct = 0;

struct AscentData{
	uint32_t OldTick;
	float OldAlt;
}ascent_data;

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
    }
    else{
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

    char lonSide = values[5][0];
    char latSide = values[3][0];
    //strcpy(nmea_data->lastMeasure, values[1]);

    char hoursc[3];
    char minutesc[3];
    char secondsc[3];

    sprintf(hoursc, "%c%c", values[1][0], values[1][1]);
    sprintf(minutesc, "%c%c", values[1][2], values[1][3]);
    sprintf(secondsc, "%c%c", values[1][4], values[1][5]);

    nmea_data->Hours = strtol(hoursc, NULL, 10);
    nmea_data->Minutes = strtol(minutesc, NULL, 10);
    nmea_data->Seconds = strtol(secondsc, NULL, 10);

    if(latSide == 'S' || latSide == 'N'){
        char lat_d[2];
        char lat_m[7];
        for (int z = 0; z < 2; z++) lat_d[z] = values[2][z];
        for (int z = 0; z < 6; z++) lat_m[z] = values[2][z + 2];

        int lat_deg_strtol = strtol(lat_d, NULL, 10);
        float lat_min_strtof = strtof(lat_m, NULL);
        double lat_deg = lat_deg_strtol + lat_min_strtof / 60;

        char lon_d[3];
        char lon_m[7];

        for (int z = 0; z < 3; z++) lon_d[z] = values[4][z];
        for (int z = 0; z < 6; z++) lon_m[z] = values[4][z + 3];

        int lon_deg_strtol = strtol(lon_d, NULL, 10);
        float lon_min_strtof = strtof(lon_m, NULL);
        double lon_deg = lon_deg_strtol + lon_min_strtof / 60;

        if(lon_deg_strtol != 0 && lon_min_strtof != 0 && lat_deg_strtol != 0 && lat_min_strtof != 0 && lat_deg!=0 && lon_deg!=0 && lat_deg<90 && lon_deg<180){
            nmea_data->Lat = lat_deg;
            nmea_data->Lon = lon_deg;
            if(latSide == 'S') nmea_data->Lat *= -1;
            if(lonSide == 'W') nmea_data->Lon *= -1;

            float altitude = strtof(values[9], NULL);

            nmea_data->Alt = altitude !=0 ? altitude : nmea_data->Alt;

            uint32_t tick = HAL_GetTick();
            if(ascent_data.OldTick !=0 && altitude != 0 && (tick-ascent_data.OldTick)>10000){
            	nmea_data->AscentRate = (altitude-ascent_data.OldAlt)/((tick-ascent_data.OldTick)/1000);
            	ascent_data.OldTick = tick;
            	ascent_data.OldAlt = nmea_data->Alt;
            }

            nmea_data->Sats = strtol(values[7], NULL, 10);

            int fixQuality = strtol(values[6], NULL, 10);
            nmea_data->Fix = fixQuality > 0 ? 1 : 0;

            float hdop = strtof(values[8], NULL);
            nmea_data->HDOP = hdop!=0 ? hdop : nmea_data->HDOP;
        }
        else {
            for(int i=0; i<counter; i++) free(values[i]);
            return 0;
        }

    }

    for(int i=0; i<counter; i++) free(values[i]);
    return 1;
}


int nmea_GSA(NMEA *nmea_data, char*inputString){
	char *values[25];
    memset(values, 0, sizeof(values));
	int counter = getValues(inputString, values);

    int fix = strtol(values[2], NULL, 10);
    nmea_data->Fix = fix > 1 ? 1 : 0;
    int satelliteCount = 0;
    for(int i=3; i<15; i++){
        if(values[i][0] != '\0'){
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

    char latSide = values[2][0];
    if (latSide == 'S' || latSide == 'N') { //check if data is sorta intact
        char lat_d[2];
        char lat_m[7];
        for (int z = 0; z < 2; z++) lat_d[z] = values[1][z];
        for (int z = 0; z < 6; z++) lat_m[z] = values[1][z + 2];

        int lat_deg_strtol = strtol(lat_d, NULL, 10);
        float lat_min_strtof = strtof(lat_m, NULL);
        double lat_deg = lat_deg_strtol + lat_min_strtof / 60;

        char lon_d[3];
        char lon_m[7];
        char lonSide = values[4][0];
        for (int z = 0; z < 3; z++) lon_d[z] = values[3][z];
        for (int z = 0; z < 6; z++) lon_m[z] = values[3][z + 3];

        int lon_deg_strtol = strtol(lon_d, NULL, 10);
        float lon_min_strtof = strtof(lon_m, NULL);
        double lon_deg = lon_deg_strtol + lon_min_strtof / 60;
        //confirm that we aren't on null island
        if(lon_deg_strtol != 0 && lon_min_strtof != 0 && lat_deg_strtol != 0 && lat_min_strtof != 0 && lat_deg!=0 && lon_deg!=0 && lat_deg<90 && lon_deg<180){
            nmea_data->Lat = lat_deg;
            nmea_data->Lon = lon_deg;
            if(latSide == 'S') nmea_data->Lat *= -1;
            if(lonSide == 'W') nmea_data->Lon *= -1;
        }
        for(int i = 0; i<counter; i++) free(values[i]);
        return 1;
    }
    else{
    	for(int i = 0; i<counter; i++) free(values[i]);
    	return 0;
    }
}

int nmea_VTG(NMEA *nmea_data, char*inputString) {
	char *values[25];
    memset(values, 0, sizeof(values));
	int counter = getValues(inputString, values);

    float speed = strtof(values[5], NULL); // 5 for kph, 3 for knots
    nmea_data->Speed = speed;

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
    }
    for(uint8_t i = 0; i<cnt; i++) free(data[i]);
}
