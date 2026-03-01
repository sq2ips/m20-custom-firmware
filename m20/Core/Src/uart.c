//
// Created by Piotr Macuk on 01/03/2026.
//

#include "uart.h"

void uart_transmit(USART_TypeDef *uart, const uint8_t *buffer, const uint16_t length) {
  // disable UART RX interrupt to not occur while the mode change
  LL_LPUART_DisableIT_RXNE(uart);
  for (uint16_t i = 0; i < length; i++) {
    while (!LL_LPUART_IsActiveFlag_TXE(uart)) { }
    LL_LPUART_TransmitData8(uart, buffer[i]);
  }
  while (!LL_LPUART_IsActiveFlag_TC(uart)) { }
  // enable UART RX interrupt
  LL_LPUART_EnableIT_RXNE(uart);
}
