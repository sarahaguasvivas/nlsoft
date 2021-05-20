#ifndef __SIGNALS_H__
#define __SIGNALS_H__

#include <stdlib.h>
#include <string>


#define RXD2 16
#define TXD2 17

void setup_signal_collector();
void collect_signal(float *, float*, int);

#endif
