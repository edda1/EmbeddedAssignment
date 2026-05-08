/*
 * File:   spi.c
 * Author: kulle
 *
 * Created on May 7, 2026, 5:44 PM
 */


#include "xc.h"
#include "spi.h"

#define PI 3.14159265358979323846



void spi_setup(){
    SPI1STATbits.SPIEN = 0;             // we disable before we do changes
    
    // I/O setup
    TRISAbits.TRISA1 = 1;               // RA1 corresponds to RPI17 and we set it as input for master-in
    TRISFbits.TRISF12 = 0;              // RF12 corresponds to RP108 SCK and we set it as output for the clock
    TRISFbits.TRISF13 = 0;              // RF13 corresponds to RP109 and we set it as output for master-out
    
    // Pin remapping
    RPINR20bits.SDI1R = 0b0010001;      // MISO (SDI1) - RPI17 [RA1]
    RPOR12bits.RP109R = 0b000101;       // MOSI (SDO1) - RP109 [RF13]  
    RPOR11bits.RP108R = 0b000110;       // SCK1 [RF12] - RP108 [RF12]

    // Modality setup
    SPI1CON1bits.MSTEN = 1;     // enable master
    SPI1CON1bits.MODE16 = 0;    // 8bit modality
    SPI1CON1bits.CKP = 1;       // set the clock idle to 1
    
    // to match the operating frequency of the slave
    SPI1CON1bits.PPRE = 3;       // primary presacler 1:1
    SPI1CON1bits.SPRE = 3;       // secondary prescaler 3:1
    
    SPI1STATbits.SPIROV = 0;    // clear overflow flag 
    SPI1STATbits.SPIEN = 1;     // enable spi  
    
    
    // Chip Select Setup
    
    // define them as output
    TRISDbits.TRISD6 = 0;       // for magnetometer     CS3
    TRISBbits.TRISB4 = 0;       // for gyroscope        CS2
    TRISBbits.TRISB3 = 0;       // for accelerometer    CS1
    
    // deselect devices (inactive state = HIGH)
    LATDbits.LATD6 = 1;         // mag
    LATBbits.LATB4 = 1;         // gyro
    LATBbits.LATB3 = 1;         // accel
    
}

unsigned int spi_write(unsigned int data){
    
    // send one byte over the SPI and wait for response
    
    while (SPI1STATbits.SPITBF == 1);    // wait until the t-buffer is emptied 
    
    SPI1BUF = data;                      // write data (also triggers the clock)
    
    while (SPI1STATbits.SPIRBF == 0);    // wait until the whole transmission is complete
    
    return SPI1BUF;     // return the data we got from the slave
}

void accel_bw(int bw){
    
    // Set accelerometer filter bandwidth (affects update rate + noise filtering)

    // boots in normal mode the moment it is selected
    LATBbits.LATB3 = 0;         // slelct accelerometer
    spi_write(0x10 & 0x7F);     // write to the bandwidth register
    
    switch(bw) {
    // each filtering frequency requires a different time to update before new data can be considered
        case 8:  spi_write(0x08); break;
        case 9:  spi_write(0x09); break;
        case 10: spi_write(0x0A); break;
        case 11: spi_write(0x0B); break;
        case 12: spi_write(0x0C); break;
        case 13: spi_write(0x0D); break;
        case 14: spi_write(0x0E); break;
        case 15: spi_write(0x0F); break;
        default: 
            // handle invalid bandwidth request
            LATBbits.LATB3 = 1;
            uart_transmit("$ERR,1*");
            return;
    }
    LATBbits.LATB3 = 1;          // deslelct accelerometer
    
      
}

Accel_DataStruct accel_read(){
    
    // SPI returns 2 bytes per axis (MSB + LSB)
    // accelerometer outputs 12-bit signed values inside 16-bit frame
    // lower 4 bits are unused, shift right
    
    Accel_DataStruct result;
    
    uint16_t LSB_part, MSB_part;
    
    // read accelerometer data
    
    // x-axis (register increments after read)
    LATBbits.LATB3 = 0;
    
    spi_write(0x02 | 0x80); 
    LSB_part = spi_write(0x00);
    MSB_part = spi_write(0x00);             
    result.axis_x = ((int16_t)((MSB_part << 8) | LSB_part)) >> 4;

    // y-axis starts at 0x04 ends at 0x05
    LSB_part = spi_write(0x00);
    MSB_part = spi_write(0x00);           
    result.axis_y = ((int16_t)((MSB_part << 8) | LSB_part)) >> 4;

    // z-axis starts at 0x06 ends at 0x07
    LSB_part = spi_write(0x00);
    MSB_part = spi_write(0x00);         
    result.axis_z = ((int16_t)((MSB_part << 8) | LSB_part)) >> 4;   // z-axis has 15 bits
    
    LATBbits.LATB3 = 1;
    
    // convert to physical units to get acceleration in g
    float ax, ay, az;    
    ax = (float) result.axis_x * 0.00098;
    ay = (float) result.axis_y * 0.00098;
    az = (float) result.axis_z * 0.00098;
    
    // angle calculation
    result.roll = atan2f(ay, az) * (180.0 / (float)PI);  // roll in degrees
    result.pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * (180.0 / (float)PI); //pitch in degrees
    
    return result;
}