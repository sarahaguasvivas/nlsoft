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

#define OLD_MIN -1
#define OLD_MAX 1
#define NEW_MIN 0
#define NEW_MAX UINT16_MAX

void step_motor(float *, int);

uint16_t convert_mapped_values(float);

#endif
