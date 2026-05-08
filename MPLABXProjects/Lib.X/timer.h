#ifndef TIMER_H
#define	TIMER_H

// timer selection
#define TIMER1 1
#define TIMER2 2

// configure timer period in ms
void tmr_setup_period(int timer, int ms);

// waiting for timer overflow
int tmr_wait_period(int timer);

// blocking delay in ms
void tmr_wait_ms(int timer, int ms);

#endif

