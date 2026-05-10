#include <stdio.h>
#include <stdlib.h>
#include "xc.h"
#include "timer.h"
#include "spi.h"
#include "uart.h"
#include <math.h>

// Scheduling counters 
static int accel_count = 0;     // Counts loop iterations between accel reads
static int char_index  = 0;     // Number of valid characters in the shift reg  
static int timer_count = 0;     // Counts iterations for the 1 Hz LED blink     
static int ang_count   = 5;     // Stagger the first $ANG* send by 50 ms        

static int hz_period = 10;      // Default 10 Hz                               
static int hz_count  = 0;

//initial_setup
static void initial_setup() {

    // Disable analog inputs 
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    tmr_setup_period(TIMER1, 10);

    TRISGbits.TRISG9 = 0;   // LED2 output
    LATGbits.LATG9 = 1;   // initial state: on
}

static void algorithm() {
    tmr_wait_ms(TIMER2, 7);
}

int main() {

    int period_misses = 0;
    int value = 0;

    char reciv_char[7] = {0};   // Shift register for command parsing, holds the last 7 received bytes
    char buffer_euler[48];
    char tempChar;

    Sensor_DataStruct ads = {0};

    initial_setup();
    uart_setup(115200);
    spi_setup();
    accel_bw(15);

    while (1) {

        algorithm();

        // Accelerometer read at 50 Hz 
        if (++accel_count >= 2) {
            accel_count = 0;
            ads = sensor_read(ACC);
        }

        //Transmit $ACC* at the configured frequency 
        if (hz_period > 0 && ++hz_count >= hz_period) {
            hz_count = 0;
            sprintf(buffer_euler, "$ACC,%d,%d,%d*",
                    (int)ads.axis_x,
                    (int)ads.axis_y,
                    (int)ads.axis_z);
            uart_transmit(buffer_euler);
        }

        //Transmit $ANG* and $DBG* at 5 Hz
        if (++ang_count >= 20) {
            ang_count = 0;

            sprintf(buffer_euler, "$ANG,%.1f,%.1f*",
                    (double)ads.roll,
                    (double)ads.pitch);
            uart_transmit(buffer_euler);

            // Debug message
            /*char dbg[32];
            sprintf(dbg, "$DBG,%d*", period_misses);
            uart_transmit(dbg);*/
        }

        //Drain the receive buffer and parse commands 
        while (uart_receive_char(&tempChar)) {

            reciv_char[0] = reciv_char[1];
            reciv_char[1] = reciv_char[2];
            reciv_char[2] = reciv_char[3];
            reciv_char[3] = reciv_char[4];
            reciv_char[4] = reciv_char[5];
            reciv_char[5] = reciv_char[6];
            reciv_char[6] = tempChar;

            if (char_index < 7) {
                char_index++;
            }

            if (char_index == 7) {
                if (reciv_char[0] == '$'  &&
                    reciv_char[3] == ','  &&
                    reciv_char[6] == '*'  &&
                    ((reciv_char[1] == 'H' && reciv_char[2] == 'Z') ||
                     (reciv_char[1] == 'B' && reciv_char[2] == 'W')) &&
                    (reciv_char[4] >= '0' && reciv_char[4] <= '9')   &&
                    (reciv_char[5] >= '0' && reciv_char[5] <= '9')) {

                    // Convert two ASCII digit characters to an integer
                    value = (reciv_char[4] - '0') * 10 + (reciv_char[5] - '0');

                    if (reciv_char[1] == 'H' && reciv_char[2] == 'Z') {
                        // $HZ,xx* ? update transmission frequency
                        hz_count  = 0;     
                        hz_period = uart_frequency_change(value, hz_period);
                    } else {
                        // $BW,xx* ? update accelerometer bandwidth filter
                        accel_bw(value);
                    }

                    reciv_char[0] = reciv_char[1] = reciv_char[2] = reciv_char[3] = 0;
                    reciv_char[4] = reciv_char[5] = reciv_char[6] = 0;
                    char_index = 0;
                }
            }
        }

        if (tmr_wait_period(TIMER1)) {
            period_misses++;
        }

        // Toggle LED2 at 1 Hz 
        if (++timer_count == 50) {
            LATGbits.LATG9 = !LATGbits.LATG9;
            timer_count = 0;
        }
    }

    return EXIT_SUCCESS;
}
