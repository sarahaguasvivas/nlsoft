/* 
 * This module is responsible for actutating the motors. 
 *     For the sleeve setup it does so by sending
 *     actuation data through Serial to the OpenCM 9.04 board 
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

void step_motor(float *, int);

uint16_t convert_mapped_values(float);

#endif
