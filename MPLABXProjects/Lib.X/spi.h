#ifndef SPI_H
#define	SPI_H


#ifdef	__cplusplus
extern "C" {
#endif 
    
#include <stdlib.h>
    
#define ACC 1
#define GYR 2
#define MAG 3
    
typedef struct{
    int16_t axis_x;
    int16_t axis_y;
    int16_t axis_z;

    float roll;
    float pitch;
} Accel_DataStruct;
     

void spi_setup();
unsigned int spi_write(unsigned int data);
void accel_bw(int bw);
Accel_DataStruct accel_read();

#ifdef	__cplusplus
}
#endif 

#endif