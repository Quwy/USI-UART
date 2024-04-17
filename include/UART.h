#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#define TX_BUFFER_SIZE 16
#define RX_BUFFER_SIZE 8

#define BAUD_RATE 9600 // allowed any value
#define TIMER_SCALE 8 // allowed only (1, 8, 64, 256, 1024)

// transmit one byte
void uart_tx(const uint8_t byte);

// transmit string
void uart_tx_str(const char *const str);

// get received bytes count
uint8_t uart_rx_count(void);

// get one byte from RX buffer
// !!! NO ACTUAL BUFFER LENGTH CHECKED !!!
// !!! CALL uart_rx_count() BEFORE !!!
uint8_t uart_rx(void);

void uart_init(void);

#endif /* __UART_H__ */
