#ifndef SPI_H
#define	SPI_H
    
#include <stdlib.h>
    
#define ACC 1
#define GYR 2
#define MAG 3
    
typedef struct{
    float axis_x;
    float axis_y;
    float axis_z;

    float roll;         //accelerometer: computed roll angle in degrees
    float pitch;        //accelerometer: computed pitch angle in degrees
} Sensor_DataStruct;
     
// Sets up SPI configuration and configures pins
void spi_setup();

// Sends byte over SPI and returns receives byte
unsigned int spi_write(unsigned int data);

// sets accellerometer bandwidth
void accel_bw(int bw);

void mag_setup(void);

// Reads accelerometer data and returns Sensor Datastruct
Sensor_DataStruct accel_read();

// Reads magnetometer data and returns Sensor Datastruct
Sensor_DataStruct mag_read();

// Allows to chose what sensor to read from
Sensor_DataStruct sensor_read(int select);


#endif