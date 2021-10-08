#include "dynamixel_functions.hpp"

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
int motor_commands[2];

void receive_data();
void toggle_data_flag();

void setup() {
  setup_dynamixels(2);
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
 // delay(6);
  receive_data();
  toggle_data_flag();

  char *ptr = NULL;
  
  ptr = strtok(receivedChars, "<,>");
  //Serial.println(ptr);
  int i = 0; 
  while (ptr != NULL){
    motor_commands[i++] = (int)atoi(ptr);
    ptr = strtok(NULL, ","); 
  }
  Serial.print(motor_commands[0]); Serial.print(",");
  Serial.println(motor_commands[1]);
}

void receive_data() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
        while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void toggle_data_flag() {
    if (newData == true) {
        newData = false;
    }
}
