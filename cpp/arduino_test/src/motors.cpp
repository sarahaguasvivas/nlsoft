#include "motors.hpp"

/* 
 *  This module is responsible of sending the motor commands through SPI to the OpenCM board
 */

void setup_motor(){
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
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
  String to_send;
  uint8_t buffer[2*sizeof(float)]; // 8 = 4 bytes (float) * 2 motors TODO(sarahaguasvivas): dynamic allocation
  for (int i = 0; i< m; i++){
      uint8_t * buff;
      buff = (uint8_t*)(&u[i]);
      for (int j = 0; j < sizeof(float); j++){
        buffer[i*sizeof(float) + j] = buff[j];        
      }
  }
  Serial2.write(buffer, 8);
}
