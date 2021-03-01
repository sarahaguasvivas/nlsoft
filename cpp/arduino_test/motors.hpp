

/* 
 *  This module is responsible of sending the motor commands through SPI to the OpenCM board
 */

#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdlib.h>
#include <SPI.h>
#include <Arduino.h>

#define OLD_MIN -100
#define OLD_MAX 50
#define NEW_MIN 0
#define NEW_MAX 600


void setup_motor();

void setup_clock_divider();

void step_motor(float *, int);

uint8_t convert_mapped_values(float);


#endif
