// UPDI_Programmer.c
// STK500 compatible UPDI programmer firmware for Arduino
// Version 1.0

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Define UPDI communication pins
#define UPDI_PORT PORTB
#define UPDI_DDR  DDRB
#define UPDI_PIN  PINB
#define UPDI_BIT  PB0

// UPDI communication constants
#define UPDI_BAUD 225000  // UPDI baud rate
#define UPDI_BIT_TIME (1000000 / UPDI_BAUD)

// Function prototypes
void uart_init(void);
void updi_init(void);
void updi_send_byte(uint8_t data);
uint8_t updi_receive_byte(void);

// Main function
int main(void) {
    // Initialize UART for communication with PC
    uart_init();
    
    // Initialize UPDI interface
    updi_init();
    
    // Main program loop
    while (1) {
        // TODO: Implement main program logic
    }
    
    return 0;
}

// Initialize UART
void uart_init(void) {
    // Set baud rate
    UBRR0H = (F_CPU / 16 / 115200 - 1) >> 8;
    UBRR0L = (F_CPU / 16 / 115200 - 1);
    
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Initialize UPDI interface
void updi_init(void) {
    // Set UPDI pin as output
    UPDI_DDR |= (1 << UPDI_BIT);
    
    // Set UPDI pin high (idle state)
    UPDI_PORT |= (1 << UPDI_BIT);
}

// Send a byte over UPDI
void updi_send_byte(uint8_t data) {
    // Start bit
    UPDI_PORT &= ~(1 << UPDI_BIT);
    _delay_us(UPDI_BIT_TIME);
    
    // Data bits
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x01) {
            UPDI_PORT |= (1 << UPDI_BIT);
        } else {
            UPDI_PORT &= ~(1 << UPDI_BIT);
        }
        data >>= 1;
        _delay_us(UPDI_BIT_TIME);
    }
    
    // Stop bits (2)
    UPDI_PORT |= (1 << UPDI_BIT);
    _delay_us(2 * UPDI_BIT_TIME);
}

// Receive a byte over UPDI
uint8_t updi_receive_byte(void) {
    uint8_t data = 0;
    
    // Wait for start bit
    while (UPDI_PIN & (1 << UPDI_BIT));
    _delay_us(UPDI_BIT_TIME / 2);
    
    // Read data bits
    for (uint8_t i = 0; i < 8; i++) {
        _delay_us(UPDI_BIT_TIME);
        data >>= 1;
        if (UPDI_PIN & (1 << UPDI_BIT)) {
            data |= 0x80;
        }
    }
    
    // Wait for stop bits
    _delay_us(2 * UPDI_BIT_TIME);
    
    return data;
}
