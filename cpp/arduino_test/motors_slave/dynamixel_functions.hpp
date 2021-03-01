#define ADDR_AX_TORQUE_ENABLE           24                
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36
#define MOVING                          46
#define PROTOCOL_VERSION                1.0                
#define BAUDRATE                        57600
#define DEVICENAME                      "3"                
#define TORQUE_ENABLE                   1                  
#define TORQUE_DISABLE                  0                   
#define DXL_MINIMUM_POSITION_VALUE      0                
#define DXL_MAXIMUM_POSITION_VALUE      600               
#define DXL_MOVING_STATUS_THRESHOLD     20                  
#define ESC_ASCII_VALUE                 0x1b

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

int goalPosition = 0;
int isMoving = 0;
int dxl_comm_result[2] = {COMM_TX_FAIL, COMM_TX_FAIL};            
uint8_t dxl_error = 0;                       
int16_t dxl_present_position = 0;          

uint8_t dynamixel_ids[2] = {1, 2};
void actuate_motors(int *);
void setup_dynamixel(int);
