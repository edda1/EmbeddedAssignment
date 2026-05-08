#ifndef UART_H
#define UART_H

typedef struct{
    volatile char *data;
    volatile int head;
    volatile int tail;
    const int buf_size;
}Circular_Buffer;


void uart_init(void);
int cb_produce(Circular_Buffer *cb, char c);
int cb_consume(Circular_Buffer *cb, char *out);
void uart_transmit(const char* message);
int uart_receive_char(char *out);
int uart_frequency_change(int value, int current);

#endif