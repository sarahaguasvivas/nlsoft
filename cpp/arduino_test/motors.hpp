/* 
 *  This module is responsible of sending the motor commands through SPI to the OpenCM board
 */

#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdlib.h>
#include <SPI.h>

digitalWrite(SS, HIGH);
SPI.begin();
SPI.setClockDivider(SPI_CLOCK_DIV8);

void step_motor(float *, int);

int convert_mapped_values(float);


#endif
