#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "UART.h"

#ifndef F_CPU
#error "F_CPU is mandatory for this code"
#endif

#define TIMER_SEED ((F_CPU) / (TIMER_SCALE) / (BAUD_RATE))

#if TIMER_SEED < 10 // minimal accuracy of the timer for 10-bit packet, lower value may cause synchronization fail on last bits
#error "Invalid parameters combination: decrease TIMER_SCALE, decrease BAUD_RATE, or increase F_CPU"
#elif TIMER_SEED >= 255
#error "Invalid parameters combination: increase TIMER_SCALE, increase BAUD_RATE, or decrease F_CPU"
#endif

typedef enum {
    STAGE_IDLE           = 0b00000000,
    STAGE_TX_FIRST_4_BIT = 0b00000001,
    STAGE_TX_LAST_4_BIT  = 0b00000010,
    STAGE_RX_FIRST_4_BIT = 0b00000100,
    STAGE_RX_LAST_4_BIT  = 0b00001000
} stage_t;

static volatile uint8_t current_byte; // buffer for current byte processed (RX or TX)
static volatile uint8_t tx_buffer[TX_BUFFER_SIZE]; // TX buffer
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE]; // RX buffer
static volatile uint8_t tx_buffer_size = 0; // Actual size of TX data
static volatile uint8_t rx_buffer_size = 0; // Actual size of RX data
static volatile stage_t stage = STAGE_IDLE; // Current machine stage

// Reverse bits of the byte
static uint8_t reverse_byte(uint8_t x) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xAA);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xCC);
    x = ((x >> 4) & 0x0F) | ((x << 4) & 0xF0);
    return x;
}

// wait for operation complete
static void wait_for(const uint8_t operations) {
    while((stage & operations) != 0);
}

// Get lower byte from buffer and shift rest of bytes left
// !!! NO ACTUAL BUFFER LENGTH CHECKED !!!
static uint8_t get_next_byte(volatile uint8_t *const buffer, volatile uint8_t *const buffer_size) {
    const uint8_t result = buffer[0]; // make a copy of the first byte
        
    --(*buffer_size); // decrease size
        
    // shift rest of the buffer left
    for(uint8_t i = 0; i < *buffer_size; ++i) {
        buffer[i] = buffer[i + 1];
    }

    return result;
}

// Append byte to the buffer end
// Free buffer space is checked before action
static bool set_next_byte(const uint8_t byte, volatile uint8_t *const buffer, volatile uint8_t *const buffer_size, const uint8_t max_buffer_size) {
    if(*buffer_size < max_buffer_size) {
        buffer[(*buffer_size)++] = byte; // write byte and increase size
        return true;
    } else {
        return false; // return false if no room
    }
}

static void stop_timer(void) {
    TCCR0B &= ~(1 << CS00) & ~(1 << CS01) & ~(1 << CS02); // Stop Timer0
}

static inline void seed_timer(const uint8_t seed) {
    TCNT0 = seed; // Reset Timer0 counter
}

static void start_timer(const uint8_t seed) {
    seed_timer(seed);

    #if TIMER_SCALE == 1 // Start Timer0 with prescaler 1/1
    TCCR0B |= (1 << CS00);
    TCCR0B &= ~(1 << CS01) & ~(1 << CS02);
    #elif TIMER_SCALE == 8 // Start Timer0 with prescaler 1/8
    TCCR0B |= (1 << CS01);
    TCCR0B &= ~(1 << CS00) & ~(1 << CS02);
    #elif TIMER_SCALE == 64 // Start Timer0 with prescaler 1/64
    TCCR0B |= (1 << CS00) | (1 << CS01);
    TCCR0B &= ~(1 << CS02);
    #elif TIMER_SCALE == 256 // Start Timer0 with prescaler 1/256
    TCCR0B |= (1 << CS02);
    TCCR0B &= ~(1 << CS00) & ~(1 << CS01);
    #elif TIMER_SCALE == 1024 // Start Timer0 with prescaler 1/1024
    TCCR0B |= (1 << CS00) | (1 << CS02);
    TCCR0B &= ~(1 << CS01);
    #else
    #error "Invalid TIMER_SCALE value, allowed only 1, 8, 64, 256, 1024"
    #endif
}

static void start_byte_tx(void) {
    stage = STAGE_TX_FIRST_4_BIT; // First stage begins

    current_byte = reverse_byte(get_next_byte(tx_buffer, &tx_buffer_size)); // extract next byte from TX buffer
        
    USIDR = (current_byte & 0b11110000) >> 1; // Start bit (0) and high half of the data byte
    USISR = (USISR & 0b11110000) | (0b00001111 & (16 - 5)); // 5 bits total
    USICR |= (1 << USIWM0); // Enable USI, connect DO pin to transmitter
}

static void start_byte_rx(void) {
    stage = STAGE_RX_FIRST_4_BIT; // First stage begins

    USIDR = 0xFF; // High bit must be always 1 during left shift by rx data
    USISR = (USISR & 0b11110000) | (0b00001111 & (16 - 5)); // 5 bits total
    USICR |= (1 << USIWM0); // Enable USI
}

// stop byte TX or RX
static void stop_byte(void) {
    USICR &= ~(1 << USIWM0); // Disable USI, disconnect DO pin from transmitter
    stage = STAGE_IDLE; // Idle stage
}

// transmit one byte
void uart_tx(const uint8_t byte) {
    wait_for(STAGE_TX_FIRST_4_BIT | STAGE_TX_LAST_4_BIT); // don't touch the TX buffer during transmission
    set_next_byte(byte, tx_buffer, &tx_buffer_size, TX_BUFFER_SIZE);
    
    if((stage & (STAGE_TX_FIRST_4_BIT | STAGE_TX_LAST_4_BIT)) == 0) { // start TX only if not already running (running RX will aborted)
        start_byte_tx();
        start_timer(0);
    }
}

// transmit string
void uart_tx_str(const char *const str) {
    if(str[0] != '\0') { // check if not empty string
        uart_tx((uint8_t) str[0]); // start TX first byte

        for(uint8_t i = 1; str[i] != '\0'; ++i) { // put rest of the string into TX buffer
            set_next_byte((uint8_t) str[i], tx_buffer, &tx_buffer_size, TX_BUFFER_SIZE);
        }
    }
}

// get received bytes count
uint8_t uart_rx_count(void) {
    wait_for(STAGE_RX_FIRST_4_BIT | STAGE_RX_LAST_4_BIT); // don't touch the RX buffer during transmission
    return rx_buffer_size;
}

// get one byte from RX buffer
// !!! NO ACTUAL BUFFER LENGTH CHECKED !!!
// !!! CALL uart_rx_count() BEFORE !!!
uint8_t uart_rx(void) {
    wait_for(STAGE_RX_FIRST_4_BIT | STAGE_RX_LAST_4_BIT); // don't touch the RX buffer during transmission
    return get_next_byte(rx_buffer, &rx_buffer_size);
}

void uart_init(void) {
    // Timer0 init
    GTCCR |= (1 << PSR0); // Reset Timer0 Prescaler
    TCCR0A |= (1 << WGM01); // CTC mode
    OCR0B = 0xFF; // Set maximum value to prevent compare match TCNT0 == OCR0B
    OCR0A = TIMER_SEED; // Default: 1000000 / 9600 = 104
    
    // Pins init
    DDRB |= (1 << DDB1); // DO pin as output
    PORTB |= (1 << PB1); // DO pin to HIGH
    GIMSK |= (1 << PCIE); // Enable pin change interrupts
    PCMSK |= (1 << PCINT0); // Enable DI pin change interrupt (PCINT0_vect)

    // USI init
    USICR |= (1 << USIOIE) | // Enable USI Counter Overflow Interrupt (USI_OVF_vect)
             (1 << USICS0); // Use Timer0 as clock source

    // Enable interrupts
    sei();
}

ISR(PCINT0_vect) { // Pin interrupt
    if(stage == STAGE_IDLE && (PINB & (1 << PB0)) == 0) { // Start RX on low DI edge and if no RX/TX already in progress
        start_byte_rx(); // Init RX process
        start_timer(TIMER_SEED / 2); // Timer will match at half of bit
    } else if((stage & (STAGE_RX_FIRST_4_BIT | STAGE_RX_LAST_4_BIT)) != 0) { // Any DI signal edge while RX in progress
        seed_timer(TIMER_SEED / 2); // Additional synchronization by bits edges, if any
    }
}

ISR(USI_OVF_vect) { // USI counter overflow interrupt
    cli(); // disable interrupts to avoid timings violation

    switch(stage) {
        case STAGE_TX_FIRST_4_BIT: // first 4 bit done
            USIDR = (current_byte << 4) | // Lower 4 bit of the data byte
                    (1 << 3) | // Stop bit (1)
                    (1 << 2); // One "1" bit still at top of the USIDR after transfer complete and will hold DO pin high
            USISR |= (0b00001111 & (16 - 5)); // 5 bits total
            stage = STAGE_TX_LAST_4_BIT; // Set next stage
            break;
            
        case STAGE_TX_LAST_4_BIT: // last 4 bit done
            if(tx_buffer_size > 0) { // there is more bytes in the TX buffer?
                start_byte_tx(); // continue with next byte
            } else {
                stop_timer(); // stop transmitting
                stop_byte();
            }
            break;
            
        case STAGE_RX_FIRST_4_BIT: // first 4 bit rx done
            current_byte = USIDR << 4; // save first 4 bits as high half of the byte (start bit is 4-th and shifted out)
            USIDR = 0xFF; // High bit must be always 1 during left shift by rx data
            USISR |= (0b00001111 & (16 - 5)); // 5 bits total in the next step
            stage = STAGE_RX_LAST_4_BIT; // set next stage
            break;
            
        case STAGE_RX_LAST_4_BIT: // last 4 bit rx done
            current_byte |= (USIDR >> 1) & 0b00001111; // save last 4 bits as low half of byte (stop bit is 0-th and shifted out)
            stop_byte(); // stop receiving
            stop_timer();
            set_next_byte(reverse_byte(current_byte), rx_buffer, &rx_buffer_size, RX_BUFFER_SIZE); // add byte to the RX buffer
            break;
            
        default: // unknown stage, resetting state
            stop_byte(); // stop any transferring
            stop_timer();
    }
    
    USISR |= (1 << USIOIF); // clear interrupt flag

    sei(); // enable interrupts
}
