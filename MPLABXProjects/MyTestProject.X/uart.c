#include "xc.h"
#include "uart.h"

// Sizes are power of to to allow for easy wrap-around
// default buffer sizes that get reconfigured depending on baudrate in the setup
#define T_BUF_SIZE 128      // Maximum needed with highest configurable baudrate
#define R_BUF_SIZE 64       // Max needed (smaller than transmit because strings are very short)

#define Fcy 72000000UL      // Instruction cycle frequency

static volatile char receive_data [R_BUF_SIZE];
static volatile char transmit_data[T_BUF_SIZE];

// Initializing circular buffers
Circular_Buffer receive_buffer  = { receive_data,  0, 0, R_BUF_SIZE  };
Circular_Buffer transmit_buffer = { transmit_data, 0, 0, T_BUF_SIZE  };

// RX interrupt
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;  

    while (U1STAbits.URXDA == 1) {
        cb_produce(&receive_buffer, U1RXREG);
    }
}

//TX interrupt
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
    
    // Set buffer sizes based on baudrate
    // RX scales with baudrate (more bytes/sec = bigger buffer needed)
    // TX can be smaller at 64 since it depends on message length, not baudrate
    // Buffer gets drained at least every 10ms so 
    // we calculate max bytes in 10 ms and round up to the next power of 2
    if (baudrate <= 9600) {
        receive_buffer.buf_size  = 16;
        transmit_buffer.buf_size = 32;
    } else if (baudrate <= 57600) {
        receive_buffer.buf_size  = 64;
        transmit_buffer.buf_size = 64;
    } else {
        // 115200 and above
        receive_buffer.buf_size  = 128;
        transmit_buffer.buf_size = 64;
    }
    
    // Reset head and tail so the buffer starts empty with the new size
    receive_buffer.head  = receive_buffer.tail  = 0;
    transmit_buffer.head = transmit_buffer.tail = 0;

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

// Queues stings in buffer and begons transmit
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

// Attempts to add byte to buffer, returns 1 if successful
int cb_produce(Circular_Buffer *cb, char c) {
    int next = (cb->head + 1) % cb->buf_size;

    if (next == cb->tail) {
        return 0;           // Buffer full, byte dropped
    }

    cb->data[cb->head] = c;
    cb->head = next;
    return 1;
}

// Attempts to remove one byte from the buffer, returns 1 on success
int cb_consume(Circular_Buffer *cb, char *out) {
    if (cb->tail == cb->head) {
        return 0;           // Buffer empty 
    }

    *out = cb->data[cb->tail];
    cb->tail = (cb->tail + 1) % cb->buf_size;
    return 1;
}

// Reads one character from the UART RX circular buffer into *out
int uart_receive_char(char *out) {
    return cb_consume(&receive_buffer, out);
}

// Converting a received frequency value into the corresponding loop period
int uart_frequency_change(int value, int current) {

    switch (value) {
        case  0: return 0;          //disable
        case  1: return 100;        //1000ms
        case  2: return 50;         //500ms
        case  5: return 20;         //200ms
        case 10: return 10;         //100ms
        default:
            // If a wrong value is received, send error message 2 and leave the frequency unchanged 
            uart_transmit("$ERR,2*");
            return current;
    }
}
