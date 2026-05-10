#include "xc.h"
#include "uart.h"

// Sizes are power of to to allow for easy wrap-around
// 64 was chosen to allow max string lenght (~45) while allowing queued transmissions when interrupts arrise
#define T_BUF_SIZE 64
#define R_BUF_SIZE 16

#define Fcy 72000000UL

static volatile char receive_data [R_BUF_SIZE];
static volatile char transmit_data[T_BUF_SIZE];

Circular_Buffer receive_buffer  = { receive_data,  0, 0, R_BUF_SIZE  };
Circular_Buffer transmit_buffer = { transmit_data, 0, 0, T_BUF_SIZE  };

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;  

    while (U1STAbits.URXDA == 1) {
        cb_produce(&receive_buffer, U1RXREG);
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0;  
    char c;

    if (cb_consume(&transmit_buffer, &c)) {
        U1TXREG = c;
    } else {
        IEC0bits.U1TXIE = 0;
    }
}

void uart_setup(uint32_t baudrate) {

    TRISDbits.TRISD11 = 1;      // RD11 as input 
    TRISDbits.TRISD0  = 0;      // RD0  as output 

    // Pin remapping
    RPINR18bits.U1RXR = 75;     // Map U1RX input  to RPI75 (RD11) 
    RPOR0bits.RP64R   = 0x01;   // Map U1TX output to RP64  (RD0) 

    U1MODEbits.BRGH = 1;
    U1BRG = (uint16_t)((Fcy / (4UL * baudrate)) - 1);

    // Enable UART module and transmitter 
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN   = 1;

    // Enable RX interrupt
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;
}


void uart_transmit(const char *message) {
    char c;

    // Fill the circular buffer with every character of the string
    while (*message) {
        while (!cb_produce(&transmit_buffer, *message)); 
        message++;
    }

    // Manually send the first byte to kick off the interrupt chain
    if (cb_consume(&transmit_buffer, &c)) {
        IEC0bits.U1TXIE = 0;
        U1TXREG = c;
        IEC0bits.U1TXIE = 1;    // Enable the uart so the rest is sent by interrupt
    }
}

int cb_produce(Circular_Buffer *cb, char c) {
    int next = (cb->head + 1) % cb->buf_size;

    if (next == cb->tail) {
        return 0;           // Buffer full, byte dropped
    }

    cb->data[cb->head] = c;
    cb->head = next;
    return 1;
}

int cb_consume(Circular_Buffer *cb, char *out) {
    if (cb->tail == cb->head) {
        return 0;           // Buffer empty 
    }

    *out = cb->data[cb->tail];
    cb->tail = (cb->tail + 1) % cb->buf_size;
    return 1;
}

int uart_receive_char(char *out) {
    return cb_consume(&receive_buffer, out);
}

int uart_frequency_change(int value, int current) {

    switch (value) {
        case  0: return 0;
        case  1: return 100;
        case  2: return 50;
        case  5: return 20;
        case 10: return 10;
        default:
            // If a wrong value is received, leave the frequency unchanged 
            uart_transmit("$ERR,2*");
            return current;
    }
}
