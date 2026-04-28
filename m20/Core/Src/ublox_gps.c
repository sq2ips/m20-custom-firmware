//
// Created by Piotr Macuk on 01/03/2026.
//

#include "ublox_gps.h"
#include "uart.h"

void ublox_init() {
  uart_transmit(LPUART1, ubx_init_buffer, UBX_INIT_BUFFER_LENGTH);
}
