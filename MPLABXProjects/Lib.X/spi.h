#ifndef SPI_H
#define	SPI_H
    
#include <stdlib.h>
    
#define ACC 1
#define GYR 2
#define MAG 3
    
typedef struct{
    int16_t axis_x;     //raw accelerometer x-axis value
    int16_t axis_y;     //raw accelerometer y-axis value
    int16_t axis_z;     //raw accelerometer z-axis value

    float roll;         //computed roll angle in degrees
    float pitch;        //computed pitch angle in degrees
} Accel_DataStruct;
     
// sets up SPI configuration and configures pins
void spi_setup();

//sends byte over SPI and returns receives byte
unsigned int spi_write(unsigned int data);

// sets accellerometer bandwidth
void accel_bw(int bw);

// reads accelerometer data and computes angles 
Accel_DataStruct accel_read();

#endif