#include "dynamixel_functions.hpp"

void setup_dynamixel(int m)
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
}

void actuate_motors(int * u){

  for (int i=0 ; i< n_motors; i++){
    
        int goal_position = u[i];
        
        packetHandler->read1ByteTxRx(portHandler, dynamixel_ids[i], MOVING, (uint8_t*)&isMoving, &dxl_error);
    
        if(isMoving == 0 ){ 
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dynamixel_ids[i], ADDR_AX_GOAL_POSITION, goal_position[i], &dxl_error);
     
        if(goalPosition == DXL_MAXIMUM_POSITION_VALUE)
          goalPosition = 0;
        else
          goalPosition = DXL_MAXIMUM_POSITION_VALUE;
      }
    
      packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
  } 
}
 
  
