/*
 * File:   timer.c
 * Author: kulle
 *
 * Created on May 7, 2026, 3:23 PM
 */



#include "timer.h"
#include "xc.h"

#define Fcy 72000000UL
#define Prescaler 256
#define Ticks_per_ms (Fcy / (Prescaler * 1000UL))

void tmr_setup_period(int timer, int ms){
    
    // The value of the Fcy = 72MHz
    // To support up to 200ms we need to use a 1:256 prescaler
    uint16_t pr = (Ticks_per_ms * ms);

    if (timer == TIMER1){
        T1CONbits.TON = 0;      // lets make sure that the timer is not working
        T1CONbits.TCS = 0;      // specify that the clock source is Fcy
        T1CONbits.TCKPS = 3;    // define prescaler as 1:256

        TMR1 = 0;               // we reset the timer 
        IFS0bits.T1IF = 0;      // we reset the flag
        
        PR1 = pr - 1;           // as it needs one cycle to see that it has reached the desired number
        T1CONbits.TON = 1;      // we start the timer now
    }
    else if(timer == TIMER2){
        T2CONbits.TON = 0; 
        T2CONbits.TCS = 0;  
        T2CONbits.TCKPS = 3;

        TMR2 = 0;     
        IFS0bits.T2IF = 0;      
        
        PR2 = pr - 1;
        T2CONbits.TON = 1;
    }
    
}


int tmr_wait_period(int timer){

    if (timer == TIMER1){
        if(IFS0bits.T1IF == 1){
            IFS0bits.T1IF = 0;          // we clear the flag before exiting early
            return 1;
        }
        while(IFS0bits.T1IF == 0);      // busy waiting
        IFS0bits.T1IF = 0;              // we reset it (perhaps it is redundant)
    }

    else if (timer == TIMER2){
        if(IFS0bits.T2IF == 1){
            IFS0bits.T2IF == 0;
            return 1;
        }
        while(IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0;
    }

    return 0;
}


void tmr_wait_ms(int timer, int ms){
   
    // here we define the lowest unit allowed -> 1ms
    tmr_setup_period(timer, 1);

    // we iterate until we reach the value inputed 
    for (int i = 0; i < ms; i++){
        tmr_wait_period(timer);
    }

    if (timer == TIMER1){
        T1CONbits.TON = 0; 
    }
    else if (timer == TIMER2){
        T2CONbits.TON = 0; 
    }
}


