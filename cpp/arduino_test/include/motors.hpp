

/* 
 *  This module is responsible of sending the motor commands through SPI to the OpenCM board
 */

#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdlib.h>
#include <Arduino.h>

#define OLD_MIN -1.75
#define OLD_MAX 0.83
#define NEW_MIN 0
#define NEW_MAX 600

#define RXD2 3
#define TXD2 1

void setup_motor();

void step_motor(float *, int);

uint16_t convert_mapped_values(float);


#endif
