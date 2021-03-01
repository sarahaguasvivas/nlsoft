#include "dynamixel_functions.hpp"

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
volatile int n_motors;
int goal_position[2];
int isMoving = 0;
int dxl_comm_result[2];            
uint8_t dxl_error;                       
int16_t dxl_present_position[2];
uint8_t dynamixel_ids[2];     

void setup_dynamixels(int m)
{
  n_motors = m;
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler= dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);

  for (int i = 0; i < m; i++){
      dxl_comm_result[i] = packetHandler->write1ByteTxRx(portHandler, dynamixel_ids[i], ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      Serial.print("Dynamixel "); Serial.print(dynamixel_ids[i]); Serial.print(" has been successfully connected \n");
  }

  for (int i = 0; i < m; i++){
      dxl_comm_result[i] = COMM_TX_FAIL; 
      goal_position[i] = 0;
      dxl_present_position[i] = 0;
      dynamixel_ids[i] = i+1;  
  }
 
}

void actuate_motors(int * u){

  for (int i=0 ; i< n_motors; i++){
        goal_position[i] = u[i];
        
        packetHandler->read1ByteTxRx(portHandler, dynamixel_ids[i], MOVING, (uint8_t*)&isMoving, &dxl_error);
    
        if(isMoving == 0 ){ 
            dxl_comm_result[i] = packetHandler->write2ByteTxRx(portHandler, dynamixel_ids[i], ADDR_AX_GOAL_POSITION, goal_position[i], &dxl_error);
     
        if(goal_position[i] == DXL_MAXIMUM_POSITION_VALUE)
          goal_position[i] = 0;
        else
          goal_position[i] = DXL_MAXIMUM_POSITION_VALUE;
      }
    
      packetHandler->read2ByteTxRx(portHandler, dynamixel_ids[i], ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position[i], &dxl_error);
  } 
}
 
  
