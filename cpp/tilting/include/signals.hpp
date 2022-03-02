#ifndef __SIGNALS_H__
#define __SIGNALS_H__

#include <Arduino.h>

#include <stdlib.h>
#include <string>
#include "Arduino.h"
#include "SRTxMCU.h"
#include <SPI.h>

#define NUM_CHANNELS 18

#define MAG_SENSOR_NUM 6  // quantity of sensors
#define PROBEPIN 23      // For timing or debugging only
#define SPISPEED 1000000 // Arduino Mega
#define LOOP_FREQUENCY 1000
#define LIS3MDL_FROM_FS_4G_TO_G (float)(6842.0)
#define LIS3MDL_FROM_FS_8G_TO_G (float)(3421.0)
#define LIS3MDL_FROM_FS_12G_TO_G (float)(2281.0)
#define LIS3MDL_FROM_FS_16G_TO_G (float)(1711.0)
#define NUM_CHARS 80

void toggle_CS_PINS(bool low_high); //declare the function before it's used
void setup_signal_collector();
void collect_signal(float *, float, int);
void toggle_CS_PINS(bool low_high);

#endif
