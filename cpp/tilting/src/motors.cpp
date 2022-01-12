#include "motors.hpp"

/* 
 * This module is responsible for actutating the motors. 
 *     For the sleeve setup it does so by sending
 *     actuation data through Serial to the OpenCM 9.04 board 
 */
DriverChannel chnl_1(1, 22, 23, 21); // id, charge pwm, drain pwm (constant), voltage monitor 
DriverChannel chnl_2(2, 20, 23, 19);
DriverChannel chnl_3(3, 17, 23, 16);
DriverChannel chnl_4(4, 14, 23, 39);
DriverChannel chnl_5(5, 38, 23, 37);
DriverChannel chnl_6(6, 36, 23, 35);

void setup_motors(){
  chnl_1.begin();
  chnl_2.begin();
  chnl_3.begin();
  chnl_4.begin();
  chnl_5.begin();
  chnl_6.begin();
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

    uint8_t buffer[m];
    for (int i = m - 1; i >=0; i--){
        buffer[i] = (uint8_t)(convert_mapped_values(u[i]));
    }
    // TODO(sarahaguasvivas): Figure out what data 
    //    it expects, i.e. why is it a pointer not a value
    // TODO(vani): If all else fails, just directly analogWrite
    chnl_1.set_data(22, &buffer[0]);
    chnl_2.set_data(20, &buffer[1]);
    chnl_3.set_data(17, &buffer[2]);
    chnl_4.set_data(14, &buffer[3]);
    chnl_5.set_data(38, &buffer[4]);
    chnl_6.set_data(36, &buffer[5]);
    chnl_1.spin(); 
    chnl_2.spin(); 
    chnl_3.spin();  
    chnl_4.spin();
    chnl_5.spin();
    chnl_6.spin();
}