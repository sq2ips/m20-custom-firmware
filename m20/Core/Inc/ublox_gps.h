//
// Created by Piotr Macuk on 01/03/2026.
//

#ifndef M20_UBLOX_GPS_H
#define M20_UBLOX_GPS_H

#include "gps.h"
#include <stdint.h>

#define GpsRxBuffer_SIZE 128

#define UBX_INIT_BUFFER_LENGTH (4 + 2 + 29 + 2) // header + length + payload + checksum
static const uint8_t ubx_init_buffer[UBX_INIT_BUFFER_LENGTH] = {
  // --- UBX Header ---
  0xB5, 0x62,         // Sync chars
  0x06, 0x8A,         // Class: CFG, ID: VALSET

  // --- Length: 4 (header payload) + 5*5 (5 kluczy x [4B key + 1B val]) = 29 = 0x1D ---
  0x1D, 0x00,

  // --- Payload header ---
  0x00,               // version = 0
  0x01,               // layers = RAM (0x01)
  0x00, 0x00,         // reserved

  // --- CFG-UART1INPROT-UBX = 1 (key: 0x10730001, L = 1 bajt) ---
  0x01, 0x00, 0x73, 0x10,
  0x01,

  // --- CFG-UART1INPROT-NMEA = 0 (key: 0x10730002) ---
  0x02, 0x00, 0x73, 0x10,
  0x00,

  // --- CFG-UART1OUTPROT-UBX = 1 (key: 0x10740001) ---
  0x01, 0x00, 0x74, 0x10,
  0x01,

  // --- CFG-UART1OUTPROT-NMEA = 0 (key: 0x10740002) ---
  0x02, 0x00, 0x74, 0x10,
  0x00,

  // --- CFG-NAVSPG-DYNMODEL = 6 (Airborne <1g) (key: 0x20110021, U1 = 1 bajt) ---
  0x21, 0x00, 0x11, 0x20,
  0x06,

  // --- Checksum CK_A, CK_B (Fletcher, liczone od Class do końca payload) ---
  0xF2, 0x57
};

void ublox_init();

#endif //M20_UBLOX_GPS_H
