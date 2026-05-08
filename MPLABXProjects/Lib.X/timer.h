#ifndef TIMER_H
#define	TIMER_H


#ifdef	__cplusplus
extern "C" {
#endif 


#define TIMER1 1
#define TIMER2 2

void tmr_setup_period(int timer, int ms);
int tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);
unsigned int spi_write(unsigned int data);


#ifdef	__cplusplus
}
#endif 

#endif

