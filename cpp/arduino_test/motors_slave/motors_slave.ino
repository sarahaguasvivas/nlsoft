/*
 * SPI slave code to move the motors
 * Compatible with 3 pin AX- and 4 pin RX- series
*/
#include <SPI.h>
#include "dynamixel_functions.hpp"
#define   m     2

byte storage [8];
uint16_t buff[m];
volatile byte pos;
volatile boolean process;


void setup() {
  pinMode(MISO, OUTPUT);
  pos = 0;
  process = false;
  Serial.begin(115200);
  while(!Serial);
  setup_dynamixels(m);

}



void loop()
{

}
