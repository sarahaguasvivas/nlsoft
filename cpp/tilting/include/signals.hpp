#ifndef __SIGNALS_H__
#define __SIGNALS_H__

#include <stdlib.h>
#include <string>
#include "Arduino.h"
#include "SRTxMCU.h"

#define NUM_CHANNELS 6

#define NUM_CHARS 80

void setup_signal_collector();
void collect_signal(float *, float*, int);

#endif
