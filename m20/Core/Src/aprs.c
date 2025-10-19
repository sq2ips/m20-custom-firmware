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
        if(pos >= sizeof(APRS_DESTINATION)-1){
            buff[pos] = APRS_SPACE_SYMBOL<<1;
        }else buff[pos] = APRS_DESTINATION[pos]<<1;
    }
    buff[pos++] = (APRS_DESTINATION_SSID<<1) | 0b11100000; // Destination adress SSID
    
    d_pos = pos;
    for(; pos-d_pos<6; pos++){ // Source adress
        if(pos-d_pos >= sizeof(APRS_CALLSING)-1){
            buff[pos] = APRS_SPACE_SYMBOL<<1;
        }else buff[pos] = APRS_CALLSING[pos-d_pos]<<1;
    }
    buff[pos++] = (APRS_SSID<<1) | 0b11100000; // Source adress SSID
    
    d_pos = pos;
    for(; pos-d_pos<6; pos++){ // Path 1
        if(pos-d_pos >= sizeof(APRS_PATH_1)-1){
            buff[pos] = APRS_SPACE_SYMBOL<<1;
        }else buff[pos] = APRS_PATH_1[pos-d_pos]<<1;
    }
    buff[pos++] = (APRS_PATH_1_SSID<<1) | 0b11100000; // Path 1 SSID

    d_pos = pos;
    for(; pos-d_pos<6; pos++){ // Path 2
        if(pos-d_pos >= sizeof(APRS_PATH_2)-1){
            buff[pos] = APRS_SPACE_SYMBOL<<1;
        }else buff[pos] = APRS_PATH_2[pos-d_pos]<<1;
    }
    buff[pos++] = (APRS_PATH_2_SSID<<1) | 0b11100001; // Path SSID (1 at end as last adress)

    
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


    uint32_t lon_base10 = Round(190463 * (180+lon));
    buff[cnt++] = (lon_base10 / 753571) + 33;
    buff[cnt++] = ((lon_base10 % 753571) / 8281) + 33;
    buff[cnt++] = ((lon_base10 % 753571) % 8281) / 91 + 33;
    buff[cnt++] = ((lon_base10 % 753571) % 8281) % 91 + 33;

    return cnt;
}

static uint8_t int_to_string(int32_t num, uint8_t *buff, uint8_t digits){
    uint8_t pos = 0;

    uint32_t a = 1;
    for(; digits>1; digits--) a*=10;

    if(num<0){
        buff[pos++] = '-';
        num*=-1;
    }
    while(a>=1){ // add altitude number in feet as string (6 digits)
        buff[pos++] = (num%(a*10))/a+'0';
        a/=10;
    }
    return pos;
}

static uint8_t encode_comment_telemetry(APRSPacket Packet, uint8_t *buff){
    uint8_t cnt = 0;
    
    buff[cnt++] = 'C'; // Packet count
    cnt += int_to_string(Packet.PacketCount, buff+cnt, 5); // 16 bit, 5 digits max

    buff[cnt++] = 'S'; // GNSS sat count
    cnt += int_to_string(Packet.Sats, buff+cnt, 2); // 2 digits max

    buff[cnt++] = 'R'; // GNSS restart count
    cnt += int_to_string(Packet.GpsResetCount, buff+cnt, 3); // 8 bit, 3 digits max

    buff[cnt++] = 'T'; // Internal temp
    cnt += int_to_string(Packet.Temp, buff+cnt, 2); // 2 digits (+ sign)

    buff[cnt++] = 'E'; // External temp
    cnt += int_to_string(Packet.ExtTemp, buff+cnt, 3); // *10, 2+1 digits (+ sign)

    buff[cnt++] = 'P'; // Pressure
    cnt += int_to_string(Packet.Press, buff+cnt, 5); // *10, 4+1 digits

    buff[cnt++] = 'V'; // Battery voltage
    cnt += int_to_string(Packet.BatVoltage, buff+cnt, 4); // *1000, 4 digits

    return cnt;
}

uint8_t encode_APRS_packet(APRSPacket Packet, uint8_t *buff){
    uint8_t info_field[APRS_MAX_INFO_LEN];
    uint8_t pos = 0;

    info_field[pos++] = '@'; // data type identifier
    
    info_field[pos++] = Packet.Hours/10+'0';
    info_field[pos++] = Packet.Hours%10+'0'; // Hours

    info_field[pos++] = Packet.Minutes/10+'0';
    info_field[pos++] = Packet.Minutes%10+'0'; // Minutes

    info_field[pos++] = Packet.Seconds/10+'0';
    info_field[pos++] = Packet.Seconds%10+'0'; // Seconds

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
    pos+=int_to_string(alt_ft, info_field+pos, 6);

#if APRS_COMMENT_TELEMETRY
    pos+=encode_comment_telemetry(Packet, info_field+pos);
#endif

#ifdef APRS_COMMENT_TEXT
    info_field[pos++] = ' '; // Space before comment;
    memcpy(info_field+pos, APRS_COMMENT_TEXT, APRS_MAX_INFO_LEN-pos);
    if(sizeof(APRS_COMMENT_TEXT)-1 > APRS_MAX_INFO_LEN-pos-1){
        pos+=APRS_MAX_INFO_LEN-pos-1;
    }else{
        pos+=sizeof(APRS_COMMENT_TEXT)-1;
    }
#endif

    return generate_ax25_frame(info_field, pos, buff);
}