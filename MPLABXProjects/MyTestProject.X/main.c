/*
 * File:   main.c
 * Author: kulle
 *
 * Created on March 5, 2026, 2:36 PM
 */


#include <stdio.h>
#include <stdlib.h>
#include "xc.h"
#include "timer.h"
#include <math.h>
#include "spi.h"
#include "uart.h"


int accel_count = 0;
int char_index  = 0;

int timer_count = 0;
int ang_count = 5;

int hz_period = 10;  
int hz_count  = 0; 


void initial_setup(){
    // disable the analog modality
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    // timers
    tmr_setup_period(TIMER1, 10);
    
    // LED2
    TRISGbits.TRISG9 = 0;       // output
    LATGbits.LATG9 = 1;         // initial value
}

void algorithm(){
    tmr_wait_ms(TIMER2, 7);
}

// Uart change


int main() {
    int period_misses = 0, value = 0;

    char reciv_char[7] = {0};
    char buffer_euler[48];
    char tempChar;

    Accel_DataStruct ads;

    initial_setup();
    uart_setup(115200);
    spi_setup();

    accel_bw(15);
    
    
    while(1){
        algorithm();

        // we have to get the accelerometer data every 20ms (50Hz) so every two iterations
        if (++accel_count >= 2){    
            accel_count = 0;
            ads =  accel_read();
        }

        // send the angle axes (also every ms value of the frequencies are a multiple of 20ms)
        if (hz_period > 0 && ++hz_count >= hz_period) {   // +2 bc one main loop is 10ms but we enter the daq every 20ms
            hz_count = 0;
            sprintf(buffer_euler, "$ACC,%d,%d,%d*",
            (int)ads.axis_x,
            (int)ads.axis_y,
            (int)ads.axis_z);

            uart_transmit(buffer_euler);
            // if the filter is still being normalized we just post the previous values
        }

        // for the angles which are transmitted a fixed 5Hz
        if (++ang_count >= 20) {
            ang_count = 0;
            sprintf(buffer_euler, "$ANG,%.1f,%.1f*", (double) ads.roll, (double) ads.pitch);
            uart_transmit(buffer_euler);

            
            // to check the number of missed periods
            char dbg[32];
            sprintf(dbg, "$DBG,%d*", period_misses);
            uart_transmit(dbg);
        }
        
        while (uart_receive_char(&tempChar)){
           
           reciv_char[0] = reciv_char[1];
           reciv_char[1] = reciv_char[2];
           reciv_char[2] = reciv_char[3];
           reciv_char[3] = reciv_char[4];
           reciv_char[4] = reciv_char[5];
           reciv_char[5] = reciv_char[6];
           reciv_char[6] = tempChar;
           
            if (char_index  < 7){
                char_index ++;
            }
           
           if (char_index  == 7){          
                if (reciv_char[0] == '$' && reciv_char[3] == ',' && reciv_char[6] == '*' &&
                   ((reciv_char[1] == 'H' &&  reciv_char[2] == 'Z') ||
                   (reciv_char[1] == 'B' && reciv_char[2] == 'W')) &&
                   (reciv_char[4] >= '0' && reciv_char[4] <= '9') &&
                   (reciv_char[5] >= '0' && reciv_char[5] <= '9')){ 
                    
                    value = (reciv_char[4] - '0') * 10 + (reciv_char[5] - '0'); // we turn chars into numbers
                    // FOR UART DATA FREQUENCY
                    if (reciv_char[1] == 'H' &&  reciv_char[2] == 'Z'){
                        hz_count = 0;
                        hz_period = uart_frequency_change(value, hz_period);
                    }
                    // FOR BANDWIDTH FILTER CONTROL
                    else{
                       accel_bw(value);
                    }
                    reciv_char[0] = reciv_char[1] = reciv_char[2] = reciv_char[3] = 0;
                    reciv_char[4] = reciv_char[5] = reciv_char[6] = 0;
                    char_index = 0;
                }
            } 
       }               
             
        
        if(tmr_wait_period(TIMER1)){
            period_misses++;
        }
        // Led
        if (++timer_count == 50){
            LATGbits.LATG9 = !LATGbits.LATG9;
            timer_count = 0;
        }
    }
    return (EXIT_SUCCESS);
}