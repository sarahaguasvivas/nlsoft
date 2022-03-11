/* 
 *  This module is responsible for moving the HASELS based on what a controller
 *   (outside of this module) solves for the inputs

 */

#ifndef __HASEL_H__
#define __HASEL_H__

#include <stdlib.h>
#include <math.h>

/*
#define VAR 1

All definitions, preprocessor directives, library imports
*/

void setup_motor();

void step_motor_hasel(float *, int);

#endif
