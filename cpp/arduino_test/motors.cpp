#include "motors.hpp"
#define OLDL_MIN -100
#define OLDL_MAX 50
#define NEW_MIN 0
#define NEW_MAX 600

/* 
 *  This module is responsible of sending the motor commands through SPI to the OpenCM board
 */
int convert_mapped_values(float angle)
{
    int new_angle;

    new_angle = (int)((angle - OLD_MIN)/(OLD_MAX - OLD_MIN)*(NEW_MAX - NEW_MIN) + NEW_MIN);
    
    return new_angle;
}

void step_motor(float * u, int m)
{
  /*
   * This function sends the first step input to the dynamixels
   *  u is the optimal control input matrix
   *  m is the number of motors
   */
   
   char buf;
   
   digitalWrite(SS, LOW);
   SPI.transfer(1);

   for (int pos=0; pos < sizeof(buf) - 1; pos++)
   {
      buf[pos] = SPI.transfer(0);
      if (buf[pos] == 0){
        break;
      }
   }
   buf[sizeof (buf) - 1] = 0;
   digitalWrite(SS, HIGH);
   
}
