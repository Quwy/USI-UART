
#include <avr/sleep.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <stdbool.h>

#include "UART.h"

static inline void mcu_init(void) {
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_IDLE);
}

static inline void do_yield(void) {
    sleep_cpu();
}

static inline void do_work(void) {
    uint8_t byte;

    while(uart_rx_count() > 0) {
        byte = uart_rx();

        // Upcase latin before send it back
        if(byte >= 'a' && byte <= 'z') {
            byte -= 'a' - 'A';
        }

        uart_tx(byte);
    }
}

int main(void) {
    mcu_init();
    uart_init();

    _delay_ms(1);
    uart_tx_str("USI-UART DEMO\r\r");

    while(true) {
        do_work();
        do_yield();
    }
}
