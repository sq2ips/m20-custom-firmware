/*
 *	aprs.c
 *	By SQ2IPS
 *  Based on https://joshuajer.red/blog/2023-01-04-APRS-and-ax25, https://www.aprs.org/doc/APRS101.PDF, https://github.com/projecthorus/sondehub-aprs-gateway/blob/main/README.md
 */

#include "aprs.h"
#include "config.h"
#include "utils.h"

#include <string.h>

static uint16_t calculateFcs(uint8_t *input_data, uint16_t len) { // Checksum calculator
  uint16_t crc = 0xFFFF;
  uint16_t crc16_table[] = {0x0000, 0x1081, 0x2102, 0x3183, 0x4204, 0x5285,
                            0x6306, 0x7387, 0x8408, 0x9489, 0xa50a, 0xb58b,
                            0xc60c, 0xd68d, 0xe70e, 0xf78f};

  uint16_t iterator = 0;
  while (len--) {
    crc =
        (crc >> 4) ^ crc16_table[(crc & 0xf) ^ (input_data[iterator] & 0xf)];
    crc =
        (crc >> 4) ^ crc16_table[(crc & 0xf) ^ (input_data[iterator] >> 4)];
    iterator++;
  }

  return (~crc);
}

static uint8_t generate_ax25_frame(uint8_t *info_field, uint8_t info_field_size, uint8_t *buff){
    uint8_t pos = 0;
    uint8_t d_pos = 0;
    
    for(; pos<6; pos++){ // Destination adress
        if(pos >= sizeof(APRS_D_CALL)-1){
            buff[pos] = APRS_SPACE_SYMBOL<<1;
        }else buff[pos] = APRS_D_CALL[pos]<<1;
    }
    buff[pos++] = (APRS_D_CALL_SSID<<1) | 0b11100000; // Destination adress SSID
    
    d_pos = pos;
    for(; pos-d_pos<6; pos++){ // Source adress
        if(pos-d_pos >= sizeof(APRS_S_CALL)-1){
            buff[pos] = APRS_SPACE_SYMBOL<<1;
        }else buff[pos] = APRS_S_CALL[pos-d_pos]<<1;
    }
    buff[pos++] = (APRS_S_CALL_SSID<<1) | 0b11100000; // Source adress SSID
    
    d_pos = pos;
    for(; pos-d_pos<6; pos++){ // Path
        if(pos-d_pos >= sizeof(APRS_PATH)-1){
            buff[pos] = APRS_SPACE_SYMBOL<<1;
        }else buff[pos] = APRS_PATH[pos-d_pos]<<1;
    }
    buff[pos++] = (APRS_PATH_SSID<<1) | 0b11100001; // Path SSID (1 at end as last adress)
    
    buff[pos++] = APRS_CONTROL_FIELD; // Control field
    buff[pos++] = APRS_PROTOCOL_ID; // Protocol ID

    memcpy(buff+pos, info_field, info_field_size); // info field
    pos+=info_field_size;

    uint16_t fcs = calculateFcs(buff, pos); // checksum

    buff[pos++] = fcs & 0xff;
    buff[pos++] = (fcs>>8) & 0xff;
    
    return pos;
}

static uint8_t compress_pos(float lat, float lon, uint8_t *buff){ // position compression
    uint32_t lat_base10 = Round(380926 * (90-lat));
    uint8_t cnt = 0;

    buff[cnt++] = (lat_base10 / 753571) + 33;
    buff[cnt++] = ((lat_base10 % 753571) / 8281) + 33;
    buff[cnt++] = ((lat_base10 % 753571) % 8281) / 91 + 33;
    buff[cnt++] = ((lat_base10 % 753571) % 8281) % 91 + 33;


    uint32_t lon_base10 = Round(190463 * (180+lat));
    buff[cnt++] = (lon_base10 / 753571) + 33;
    buff[cnt++] = ((lon_base10 % 753571) / 8281) + 33;
    buff[cnt++] = ((lon_base10 % 753571) % 8281) / 91 + 33;
    buff[cnt++] = ((lon_base10 % 753571) % 8281) % 91 + 33;

    return cnt;
}

uint8_t encode_APRS_packet(APRSPacket Packet, uint8_t *buff){
    uint8_t info_field[APRS_MAX_INFO_LEN];
    uint8_t pos = 0;

    info_field[pos++] = '@'; // data type identifier
    
    info_field[pos++] = Packet.Hours%10+'0'; // Hours
    info_field[pos++] = Packet.Hours/10+'0';

    info_field[pos++] = Packet.Minutes%10+'0'; // Minutes
    info_field[pos++] = Packet.Minutes/10+'0';

    info_field[pos++] = Packet.Seconds%10+'0'; // Seconds
    info_field[pos++] = Packet.Seconds/10+'0';

    info_field[pos++] = 'z'; // Zullu time

    info_field[pos++] = APRS_SYMBOL[0]; // Symbol table ID

    pos += compress_pos(Packet.Lat, Packet.Lon, info_field+pos); // Compressed position

    info_field[pos++] = APRS_SYMBOL[1]; // Symbol

    info_field[pos++] = 33; // no course data

    info_field[pos++] = Round((Log((Packet.Speed/KNOTS_TO_KMPH)+1)/Log(1.08f)))+33; // Compress speed in knots

    info_field[pos++] = 67; // Compression type

    info_field[pos++] = '/'; // Altitude header
    info_field[pos++] = 'A';
    info_field[pos++] = '=';
    uint32_t alt_ft = (uint32_t)Round(Packet.Alt/FEET_TO_M); // convert m to feet
    uint32_t a = 1000000;
    while(a>=10){ // add altitude number in feet as string
        info_field[pos++] = (alt_ft%(a*10))/a+'0';
        a/=10;
    }

    memcpy(info_field+pos, "test", 4);
    pos+=4;

    return generate_ax25_frame(info_field, pos, buff);
}