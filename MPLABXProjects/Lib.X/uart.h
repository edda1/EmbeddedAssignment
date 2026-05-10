#ifndef UART_H
#define UART_H

typedef struct{
    volatile char *data;    //buffer array
    volatile int head;      // read index  
    volatile int tail;      // write index
    int buf_size;     // buffer length
}Circular_Buffer;

// initialize uart (pins, bauderate)
void uart_setup(uint32_t baudrate);

// add char to buffer
int cb_produce(Circular_Buffer *cb, char c);

// get char from buffer
int cb_consume(Circular_Buffer *cb, char *out);

// send string over uart
void uart_transmit(const char* message);

// read received char
int uart_receive_char(char *out);

// change frequency according to input
int uart_frequency_change(int value, int current);

#endif