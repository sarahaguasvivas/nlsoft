#include "motors.hpp"

/* 
 *  This module is responsible of sending the motor commands through SPI to the OpenCM board
 */
void setup_motor(){
  //Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

uint16_t convert_mapped_values(float angle)
{
    int new_angle;

    new_angle = (uint16_t)((angle - OLD_MIN)/(OLD_MAX - OLD_MIN)*(NEW_MAX - NEW_MIN) + NEW_MIN);
    
    return new_angle;
}

void step_motor(float * u, const int m)
{
  /*
   * This function sends the first step input to the board that
   *  connects the dynamixels (OpenCM)
   *  u is the optimal control input matrix
   *  m is the number of motors
   */
  int buffer[m]; // 8 = 4 bytes (float) * 2 motors TODO(sarahaguasvivas): dynamic allocation
  for (int i = 0; i< m; i++){
      if (!isnan(u[i])){
        buffer[i] = (int)convert_mapped_values(u[i]);
      } else{
        buffer[i] = 0;
      }
  }
  char to_send[50];
  sprintf(to_send, "<%d,%d>\n", buffer[0], buffer[1]);
  Serial.print(to_send);
}
