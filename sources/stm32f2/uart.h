/*
 * UART interface implementation for STM32L microcontrollers.
 *
 * Copyright (C) 2018 Dmitry Podkhvatilin <vatilin@gmail.com>
 *
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You can redistribute this file and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software Foundation;
 * either version 2 of the License, or (at your discretion) any later version.
 * See the accompanying file "COPYING.txt" for more details.
 *
 * As a special exception to the GPL, permission is granted for additional
 * uses of the text contained in this file.  See the accompanying file
 * "COPY-UOS.txt" for details.
 */
#ifndef _STM32F4_UART_H_
#define _STM32F4_UART_H_

#include <uart/uart-interface.h>

#ifndef UART_RXBUFSZ
#define UART_RXBUFSZ	128
#endif

typedef struct _stm32f4_uart_t stm32f4_uart_t;

struct _stm32f4_uart_t {
    uartif_t            uartif;
    
    int                 port;
    
    mutex_t             rxne_mutex;
    
    USART_t             *reg;
    DMA_STREAM_t        *tx_dma;
    DMA_STREAM_t        *rx_dma;
   
    uint8_t             rx_buffer[UART_RXBUFSZ];
    uint8_t             *rx_curp;
};

int stm32f4_uart_init(stm32f4_uart_t* uart, unsigned port);

#endif // _STM32F4_UART_H_
