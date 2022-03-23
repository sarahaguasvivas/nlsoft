#include "motors.hpp"
#include <Arduino.h>
/* 
 * This module is responsible for actutating the motors. 
 *     For the sleeve setup it does so by sending
 *     actuation data through Serial to the OpenCM 9.04 board 
 */
union Convert{
    uint32_t bytes;
    float fpn;
};

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

uint8_t convert_mapped_values(float angle)
{
    int new_angle;

    new_angle = (uint8_t)((angle - OLD_MIN)/(OLD_MAX - OLD_MIN)*(NEW_MAX - NEW_MIN) + NEW_MIN);
    
    return new_angle;
}

float saturate(float u, int min_val, int max_val){
    return min(max_val, max(u, min_val));
}

void step_motor(float * u, const int m) 
{
  /*
   *  This function sends the first step input to the board that
   *  connects the dynamixels (OpenCM)
   *  u is the optimal control input matrix
   *  m is the number of motors
   */

    uint8_t buffer[m];
    for (int i = 0; i < m; i++){
        buffer[i] = (uint8_t)convert_mapped_values(u[i]/3.);
    }

    chnl_1.charge_pwm = buffer[0];
    chnl_2.charge_pwm = buffer[1];   
    chnl_3.charge_pwm = buffer[2]; 
    chnl_4.charge_pwm = buffer[3]; 
    chnl_5.charge_pwm = buffer[4]; 
    chnl_6.charge_pwm = buffer[5]; 
    
    chnl_1.update_pwm = true; 
    chnl_2.update_pwm = true; 
    chnl_3.update_pwm = true;  
    chnl_4.update_pwm = true;
    chnl_5.update_pwm = true;
    chnl_6.update_pwm = true;
    
    chnl_1.spin(); 
    chnl_2.spin(); 
    chnl_3.spin();  
    chnl_4.spin();
    chnl_5.spin();
    chnl_6.spin();

    //uint8_t d0, d1, d2, d3, d4, d5;
    //uint8_t c0, c1, c2, c3, c4, c5;

    //Convert d00, d11, d22, d33, d44, d55;
    //chnl_1.get_data(TGT_DRN_PWM, &d0); 
    //chnl_2.get_data(TGT_DRN_PWM, &d1); 
    //chnl_3.get_data(TGT_DRN_PWM, &d2);  
    //chnl_4.get_data(TGT_DRN_PWM, &d3);
    //chnl_5.get_data(TGT_DRN_PWM, &d4);
    //chnl_6.get_data(TGT_DRN_PWM, &d5);
    
    //chnl_1.get_data(TGT_CHG_PWM, &c0); 
    //chnl_2.get_data(TGT_CHG_PWM, &c1); 
    //chnl_3.get_data(TGT_CHG_PWM, &c2);  
    //chnl_4.get_data(TGT_CHG_PWM, &c3);
    //chnl_5.get_data(TGT_CHG_PWM, &c4);
    //chnl_6.get_data(TGT_CHG_PWM, &c5);

    //Serial.print(d0); Serial.print(" "); 
    //Serial.print(d1); Serial.print(" ");
    //Serial.print(d2); Serial.print(" ");
    //Serial.print(d3); Serial.print(" ");
    //Serial.print(d4); Serial.print(" ");
    //Serial.print(d5); Serial.print(" ");

    //Serial.print(c0); Serial.print(" "); 
    //Serial.print(c1); Serial.print(" ");        
    //Serial.print(c2); Serial.print(" ");
    //Serial.print(c3); Serial.print(" ");
    //Serial.print(c4); Serial.print(" ");
    //Serial.print(c5); Serial.print(" ");
    //Serial.println();
    delay(2);
}