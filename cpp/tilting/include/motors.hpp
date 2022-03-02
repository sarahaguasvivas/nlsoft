/* 
 * This module is responsible for actutating the motors. 
 *     For the sleeve setup it does so by sending
 *     actuation data through Serial to the OpenCM 9.04 board 
 */

#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdlib.h>
#include <Arduino.h>
#include "SRTxMCU.h"

#define OLD_MIN -0.5
#define OLD_MAX 0.5
#define NEW_MIN 0
#define NEW_MAX UINT8_MAX


void setup_motors();
void step_motor(float *, int, float, float);
uint8_t convert_mapped_values(float);

#endif
