#include "motors.hpp"

/* 
 *  This module is responsible of sending the motor commands through SPI to the OpenCM board
 */

void setup_motor(){
 digitalWrite(SS, HIGH);
 SPI.begin();
}

void setup_clock_divider(){
  SPI.setClockDivider(SPI_CLOCK_DIV8);
}

uint8_t convert_mapped_values(float angle)
{
    int new_angle;

    new_angle = (uint8_t)((angle - OLD_MIN)/(OLD_MAX - OLD_MIN)*(NEW_MAX - NEW_MIN) + NEW_MIN);
    
    return new_angle;
}

void step_motor(float * u, int m)
{
  /*
   * This function sends the first step input to the board that
   *  connects the dynamixels (OpenCM)
   *  u is the optimal control input matrix
   *  m is the number of motors
   */
  uint8_t buff[m];
  digitalWrite(SS, LOW);
  for (int i = 0; i < m; i++) buff[i] = convert_mapped_values(u[i]);
  SPI.transfer(buff, (uint32_t)(m*sizeof(uint16_t)));
  digitalWrite(SS, HIGH);
  
  
     
}
