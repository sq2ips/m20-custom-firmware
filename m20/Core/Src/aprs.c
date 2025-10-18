/*
 *	aprs.c
 *	By SQ2IPS
 *  Based on https://joshuajer.red/blog/2023-01-04-APRS-and-ax25, https://www.aprs.org/doc/APRS101.PDF, https://github.com/projecthorus/sondehub-aprs-gateway/blob/main/README.md
 */

#include "aprs.h"
#include "config.h"

#include <string.h>

static uint16_t calculateFcs(uint8_t *input_data, uint16_t len) {
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

static uint16_t generate_ax25_frame(uint8_t *info_field, uint8_t info_field_size, uint8_t *buff){
    volatile uint16_t pos = 0;
    uint8_t d_pos = 0;

    memset(buff, 0, sizeof(250));
    
    for(; pos<6; pos++){ // Destination adress
        buff[pos] = APRS_D_CALL[pos]<<1;
    }
    buff[pos++] = (APRS_D_CALL_SSID<<1) | 0b11100000; // Destination adress SSID
    
    d_pos = pos;
    for(; pos-d_pos<6; pos++){ // Source adress
        buff[pos] = APRS_S_CALL[pos-d_pos]<<1;
    }
    buff[pos++] = (APRS_S_CALL_SSID<<1) | 0b11100000; // Source adress SSID
    
    d_pos = pos;
    for(; pos-d_pos<6; pos++){ // Path
        buff[pos] = APRS_PATH[pos-d_pos]<<1;
    }
    buff[pos++] = (APRS_PATH_SSID<<1) | 0b11100001; // Path SSID (1 at end as last adress)
    
    buff[pos++] = APRS_CONTROL_FIELD; // Control field
    buff[pos++] = APRS_PROTOCOL_ID; // Protocol ID

    memcpy(buff+pos, info_field, info_field_size);
    pos+=info_field_size;

    uint16_t fcs = calculateFcs(buff, pos);

    buff[pos++] = fcs & 0xff;
    buff[pos++] = (fcs>>8) & 0xff;
    
    return pos;
}

uint16_t encode_APRS_packet(APRSPacket Packet, uint8_t *buff){
    uint8_t info[] = "@092345z/:*E\";qZ=OMRC/A=088132Hello World!";
    return generate_ax25_frame(info, sizeof(info)-1, buff);
}