/*
 * File:   uart.c
 * Author: kulle
 *
 * Created on 8. Mai 2026, 09:42
 */


#include "xc.h"
#include "uart.h"


#define T_BUF_SIZE 64       //tx buffer: needs to be large enough to accomodate formatted strings
#define R_BUF_SIZE 16       //rx buffer:  small, because commands are short ($HZxx, $BWxx)

#define Fcy 72000000UL     // CPU frequency in Hz

static volatile char receive_data[R_BUF_SIZE];
static volatile char transmit_data[T_BUF_SIZE];

Circular_Buffer receive_buffer = {receive_data, 0, 0, R_BUF_SIZE };
Circular_Buffer transmit_buffer = {transmit_data, 0, 0, T_BUF_SIZE };

// interrupt for reading from the uart
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;
    
    while (U1STAbits.URXDA == 1) {
        cb_produce(&receive_buffer, U1RXREG);
    }
}

// uart transmit interrupt
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0;
    char c;

    if (cb_consume(&transmit_buffer, &c)) {
        U1TXREG = c;
    } else {
        IEC0bits.U1TXIE = 0;       // nothing left so we disable interrupt
    }
}

// sets up uart and configures bauderate/communication speed
void uart_setup(uint32_t baudrate){
    // I/O for Uart
    TRISDbits.TRISD11 = 1;      // in
    TRISDbits.TRISD0 = 0;       // out   
    
    // Pin remapping
    RPINR18bits.U1RXR = 75;     // for input which is supposed to be the RD11 pin        
    RPOR0bits.RP64R = 0x01;     //  for output which is supposed to be the RD0        
    
    // Baud Rate Setup
    
    U1MODEbits.BRGH = 1;    // to select the 16 divisor
    U1BRG = (Fcy / (4UL * baudrate)) - 1;         // we load it so we can get baudrate of 115200
    
    // Power on the module
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
    
    // Enable transmitter
    IFS0bits.U1RXIF = 0;    // interrupt flag
    IEC0bits.U1RXIE = 1;    // interrupt enable
}

// send string via uart
void uart_transmit(const char* message){
    char c;

    while (*message) {
        while (!cb_produce(&transmit_buffer, *message));  // wait if buffer full
        message++;
    }

    // we manually send the first byte to kick off the interrupt chain
    if (cb_consume(&transmit_buffer, &c)) {
        IEC0bits.U1TXIE = 0;                // disable interrupt briefly
        U1TXREG = c;
        IEC0bits.U1TXIE = 1;                // we enable the uart so the rest is sent by interrupt
    }
}

// adds characters to circular buffer
int cb_produce(Circular_Buffer *cb, char c){
    int next = (cb->head + 1) % cb->buf_size;

    if (next == cb -> tail){    // if the buffer is full
        return 0;   
    }

    cb->data[cb->head] = c;
    cb->head = next;
    return 1;       
}

// removes character from circular buffer
int cb_consume(Circular_Buffer *cb, char *out){
    if (cb->tail == cb->head){
        return 0;       // transmit buffer is empty
    }
    *out = cb->data[cb->tail];
    cb->tail = (cb->tail + 1) % cb->buf_size;
    return 1;
}

// helper function to read one received byte
int uart_receive_char(char *out){
    return cb_consume(&receive_buffer, out);
}

// we change the transmit frequency of the data by determining a value that can be used inside the main loop 
int uart_frequency_change(int value, int current){
    
    int period_hz = current;
    
    switch(value) {
        case 0:  period_hz = 0;   break;    // disable
        case 1:  period_hz = 100; break;    // 1000ms
        case 2:  period_hz = 50;  break;    // 500ms
        case 5:  period_hz = 20;  break;    // 200ms
        case 10: period_hz = 10;  break;    // 100ms
        default:
            uart_transmit("$ERR,2*");
            break;
    }   
    return period_hz;
}