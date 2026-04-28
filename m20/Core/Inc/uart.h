//
// Created by Piotr Macuk on 01/03/2026.
//

#ifndef M20_UART_H
#define M20_UART_H

#include "stm32l0xx_ll_lpuart.h"
#include <stdint.h>

void uart_transmit(USART_TypeDef *uart, const uint8_t *buffer, const uint16_t length);

#endif //M20_UART_H
