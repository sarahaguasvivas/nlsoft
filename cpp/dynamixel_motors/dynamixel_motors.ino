#include <DynamixelSDK.h>
#define DEVICENAME      "3"   //Use Serial3 port
#define DXL_ID           1 

void setup() {

    dynamixel::PortHandler *portHandler =  dynamixel::PortHandler::getPortHandler(DEVICENAME);
    portHandler->openPort();

    
}

void loop() {


}
