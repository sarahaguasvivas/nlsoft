/* 
 *  This module is responsible of sending the motor commands through SPI to the OpenCM board
 */

#ifndef __HASEL_H__
#define __HASEL_H__

#include <stdlib.h>
#include <Arduino.h>

/*
#define VAR 1

All definitions, preprocessor directives, library imports
*/

void setup_motor();

void step_motor(float *, int);

#endif
