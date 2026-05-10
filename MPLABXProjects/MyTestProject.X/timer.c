#include "xc.h"
#include "timer.h"

#define Fcy 72000000UL
#define Prescaler 256

// timer tick frequency after prescaler per 1ms
#define Ticks_per_ms (Fcy / (Prescaler * 1000UL))

void tmr_setup_period(int timer, int ms){
    
    // The value of the Fcy = 72MHz
    // To support up to 200ms we need to use a 1:256 prescaler
    uint16_t pr = (Ticks_per_ms * ms);

    if (timer == TIMER1){
        T1CONbits.TON = 0;      // lets make sure that the timer is not working
        T1CONbits.TCS = 0;      // specify that the clock source is Fcy
        T1CONbits.TCKPS = 3;    // define prescaler as 1:256

        TMR1 = 0;               // reset the timer 
        IFS0bits.T1IF = 0;      // clear interrupt flag
        
        PR1 = pr - 1;           // as it needs one cycle to see that it has reached the desired number
        T1CONbits.TON = 1;      // start the timer
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
        IFS0bits.T1IF = 0;              // we reset the flag
    }

    else if (timer == TIMER2){
        if(IFS0bits.T2IF == 1){
            IFS0bits.T2IF = 0;
            return 1;
        }
        while(IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0;
    }

    return 0;
}


void tmr_wait_ms(int timer, int ms){
   
    // Blocking a delay based on repeated 1ms timer ticks
    // Reconfigures lowest unit allowed -> 1ms
    tmr_setup_period(timer, 1);

    // iterate until input value is reached
    for (int i = 0; i < ms; i++){
        tmr_wait_period(timer);
    }
    // stop timer after the delay
    if (timer == TIMER1){
        T1CONbits.TON = 0; 
    }
    else if (timer == TIMER2){
        T2CONbits.TON = 0; 
    }
}


