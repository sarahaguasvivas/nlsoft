/*
 * SPI slave code to move the motors
 * Compatible with 3 pin AX- and 4 pin RX- series
*/
#include <SPI.h>
#include "dynamixel_functions.hpp"

#define m 2

byte storage [8];
uint16_t buff[m];
volatile byte pos;
volatile boolean process;

void setup() {
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);
  pos = 0;
  process = false;
  Serial.begin(115200);
  while(!Serial);
  setup_dynamixels(m);
}

void ISR(uint16_t SPI_STC_vect)
{
  byte gathered = SPDR;
  if( pos < sizeof storage)
    {
      storage[pos++] = gathered;
    }
    else 
    process = true;
}

void loop()
{
  if( process )
  {
    Serial.print("storage[0] = "); Serial.println(storage[0]);
    Serial.print("storage[1] = "); Serial.println(storage[1]);
    Serial.print("storage[2] = "); Serial.println(storage[2]);
    Serial.print("storage[3] = "); Serial.println(storage[3]);
    Serial.print("storage[4] = "); Serial.println(storage[4]);
    Serial.print("storage[5] = "); Serial.println(storage[5]);
    Serial.print("storage[6] = "); Serial.println(storage[6]);
    Serial.print("storage[7] = "); Serial.println(storage[7]);
    memcpy(buff,&storage,8);
    Serial.print("This is buff[0]");Serial.println(buff[0]);
    Serial.print("This is buff[1]");Serial.println(buff[1]);
    storage[pos] = 0;
    pos = 0;
    process = false;
  }
}
