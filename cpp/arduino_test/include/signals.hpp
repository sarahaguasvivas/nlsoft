#ifndef __SIGNALS_H__
#define __SIGNALS_H__

#include <stdlib.h>
#include <string>
#include "Arduino.h"

#define NUM_CHANNELS 11
#define RXD2_S 16
#define TXD2_S 17

const byte numChars = 150;
char receivedCharsS[numChars];
boolean newData = false;
int signals[NUM_CHANNELS];

void receive_data();
void toggle_data_flag();
void setup_signal_collector();
void collect_signal(float *, float*, int);

#endif
