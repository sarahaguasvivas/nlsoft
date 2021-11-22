#ifndef __DYNAMIXFUNCT_H_
#define __DYNAMIXFUNCT_H_

#include <DynamixelSDK.h>

#define ADDR_AX_TORQUE_ENABLE           64                
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36
#define MOVING                          46
#define PROTOCOL_VERSION                1.0                
#define BAUDRATE                        2000000 //57600
#define DEVICENAME                      "3"                
#define TORQUE_ENABLE                   1                  
#define TORQUE_DISABLE                  0                   
#define DXL_MINIMUM_POSITION_VALUE      0                
#define DXL_MAXIMUM_POSITION_VALUE      600               
#define DXL_MOVING_STATUS_THRESHOLD     20                  
#define ESC_ASCII_VALUE                 0x1b


void actuate_motors(int *);
void setup_dynamixels(int);

#endif
