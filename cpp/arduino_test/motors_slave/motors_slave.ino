/*
 * SPI slave code to move the motors
 * Compatible with 3 pin AX- and 4 pin RX- series
*/
#include "dynamixel_functions.hpp"
#define   m     2

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  while(!Serial);
  setup_dynamixels(m);

}

void loop(){
  // when you typed any character in terminal
  if(Serial1.available()){
    //print it out though USART2(RX2,TX2)
    Serial.print((char)Serial1.read());
  }
}
