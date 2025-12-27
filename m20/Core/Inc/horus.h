#ifndef INC_HORUS_H_
#define INC_HORUS_H_

#include <stdint.h>

#define HORUS_CODED_SIZE 65

#define HORUS_INTERLEAVER 1
#define HORUS_SCRAMBLER 1

typedef struct TBinaryPacket { // https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-Packet
	uint16_t PayloadID;
	uint16_t PacketCount;
	uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;
	float Lat;
	float Lon;
	uint16_t Alt;
	uint8_t Speed;
	uint8_t Sats;
	int8_t Temp;
	uint8_t BatVoltage;
	// Custom data, 9 bytes RS41ing data
	int16_t AscentRate; // positive (raising) or negative (falling) vertical speed value in cm/s
	int16_t ExtTemp;
	uint8_t Hum;
	uint16_t Press;
	uint8_t GpsResetCount;  // counter of GPS watchdog resets
	uint8_t PvVoltage; // Payload / PV voltage
	// End of custom data
	uint16_t Checksum;
} __attribute__((packed)) HorusBinaryPacket;

uint16_t crc16(char* string, uint8_t len);

int horus_l2_encode_tx_packet(unsigned char* output_tx_data, unsigned char* input_payload_data, int num_payload_data_bytes);

void print_hex(char* data, uint8_t length, char* tmp);

#endif /* INC_HORUS_H_ */
