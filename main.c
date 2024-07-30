// UPDI_Programmer.c
// STK500 compatible UPDI programmer firmware for Arduino
// Version 1.2

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

// Define UPDI communication pins
#define UPDI_PORT PORTB
#define UPDI_DDR  DDRB
#define UPDI_PIN  PINB
#define UPDI_BIT  PB0

// UPDI communication constants
#define UPDI_BAUD 225000  // UPDI baud rate
#define UPDI_BIT_TIME (1000000 / UPDI_BAUD)

// UPDI constants
#define UPDI_SYNCH 0x55
#define UPDI_ACK   0x40

// STK500 constants
#define MESSAGE_START 0x1B
#define TOKEN 0x0E

// STK500 commands
#define CMD_SIGN_ON 0x01
#define CMD_GET_PARAMETER 0x41
#define CMD_SET_PARAMETER 0x42
#define CMD_ENTER_PROGMODE 0x50
#define CMD_LEAVE_PROGMODE 0x51

// STK500 status constants
#define STATUS_CMD_OK 0x00
#define STATUS_CMD_FAILED 0xC0

// Buffer sizes
#define MAX_BUFFER_SIZE 275

// Function prototypes
void uart_init(void);
void uart_send_byte(uint8_t data);
uint8_t uart_receive_byte(void);
void updi_init(void);
void updi_send_byte(uint8_t data);
uint8_t updi_receive_byte(void);
void updi_send_break(void);
uint8_t updi_sync(void);
void enable_system_clock_output(void);
void handle_sync_error(uint8_t attempt);
void process_stk500_command(void);
void stk500_send_response(uint8_t status, uint8_t *data, uint16_t len);

// Global variables
uint8_t rx_buffer[MAX_BUFFER_SIZE];
uint8_t tx_buffer[MAX_BUFFER_SIZE];

// Main function
int main(void) {
    // Initialize UART for communication with PC
    uart_init();
    
    // Initialize UPDI interface
    updi_init();
    
    // Main program loop
    while (1) {
        process_stk500_command();
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

// Send a byte over UART
void uart_send_byte(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

// Receive a byte over UART
uint8_t uart_receive_byte(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
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

// Send a UPDI break signal
void updi_send_break(void) {
    // Set UPDI pin as output
    UPDI_DDR |= (1 << UPDI_BIT);
    
    // Pull UPDI line low for 24.6 ms (minimum break time: 24.576 ms)
    UPDI_PORT &= ~(1 << UPDI_BIT);
    _delay_ms(24.6);
    
    // Release UPDI line
    UPDI_PORT |= (1 << UPDI_BIT);
    
    // Wait for 2 bit times (required idle periods)
    _delay_us(2 * UPDI_BIT_TIME);
}

// Perform UPDI synchronization
uint8_t updi_sync(void) {
    uint8_t response;
    uint8_t attempts = 0;
    const uint8_t max_attempts = 5;

    while (attempts < max_attempts) {
        // Send break
        updi_send_break();

        // Send synchronization character
        updi_send_byte(UPDI_SYNCH);

        // Receive response
        response = updi_receive_byte();

        if (response == UPDI_ACK) {
            return 1; // Synchronization successful
        }

        attempts++;
        handle_sync_error(attempts);
    }

    return 0; // Synchronization failed
}

// Helper function to enable system clock output (for debugging)
void enable_system_clock_output(void) {
    // Enable system clock output on PB0 (Arduino Uno pin 8)
    DDRB |= (1 << PB0);
    ASSR &= ~(1 << EXCLK);
    ASSR |= (1 << EXTCLK);
}

// Advanced error handling function using Exponential Backoff
void handle_sync_error(uint8_t attempt) {
    uint16_t backoff_time = (1 << attempt) * 100; // Exponential backoff in milliseconds
    if (backoff_time > 5000) backoff_time = 5000; // Cap at 5 seconds

    // TODO: Implement error logging or reporting mechanism
    
    _delay_ms(backoff_time);
}

// Process STK500 commands
void process_stk500_command(void) {
    uint8_t seq_num, cmd;
    uint16_t rx_len = 0;

    // Wait for message start
    while (uart_receive_byte() != MESSAGE_START);

    // Receive sequence number
    seq_num = uart_receive_byte();

    // Receive message length
    rx_len = uart_receive_byte() << 8;
    rx_len |= uart_receive_byte();

    // Receive token
    if (uart_receive_byte() != TOKEN) {
        // Invalid token, discard message
        return;
    }

    // Receive command
    cmd = uart_receive_byte();

    // Receive message body
    for (uint16_t i = 0; i < rx_len; i++) {
        rx_buffer[i] = uart_receive_byte();
    }

    // Process command
    switch (cmd) {
        case CMD_SIGN_ON:
            {
                uint8_t sign_on_response[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 'A', 'V', 'R', 'I', 'S', 'P', '_', 'M', 'K', '2'};
                stk500_send_response(STATUS_CMD_OK, sign_on_response, sizeof(sign_on_response));
            }
            break;

        case CMD_GET_PARAMETER:
            // TODO: Implement parameter retrieval
            stk500_send_response(STATUS_CMD_FAILED, NULL, 0);
            break;

        case CMD_SET_PARAMETER:
            // TODO: Implement parameter setting
            stk500_send_response(STATUS_CMD_FAILED, NULL, 0);
            break;

        case CMD_ENTER_PROGMODE:
            if (updi_sync()) {
                stk500_send_response(STATUS_CMD_OK, NULL, 0);
            } else {
                stk500_send_response(STATUS_CMD_FAILED, NULL, 0);
            }
            break;

        case CMD_LEAVE_PROGMODE:
            // TODO: Implement leaving programming mode
            stk500_send_response(STATUS_CMD_OK, NULL, 0);
            break;

        default:
            stk500_send_response(STATUS_CMD_FAILED, NULL, 0);
            break;
    }
}

// Send STK500 response
void stk500_send_response(uint8_t status, uint8_t *data, uint16_t len) {
    uint16_t tx_len = len + 2; // status + data + 2 bytes for len itself

    uart_send_byte(MESSAGE_START);
    uart_send_byte(0); // Sequence number, always 0 for responses
    uart_send_byte(tx_len >> 8);
    uart_send_byte(tx_len & 0xFF);
    uart_send_byte(TOKEN);
    uart_send_byte(status);

    for (uint16_t i = 0; i < len; i++) {
        uart_send_byte(data[i]);
    }
}
