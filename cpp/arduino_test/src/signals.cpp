#include "signals.hpp"

const byte numChars = NUM_CHARS;
char receivedCharsS[numChars];
boolean newDataS = false;
int signals[NUM_CHANNELS];

void setup_signal_collector(){
  Serial2.begin(2000000);
}

void collect_signal(float * signal_to_read, float* calibration_vector, 
                                            int signal_size)
{
  receive_data();
  toggle_data_flag();
  String recv_str = String(receivedCharsS);
  char *ptr = NULL;
  int index = recv_str.indexOf('<');
  unsigned int length_delete = (unsigned int)index - 1;
  recv_str.remove(index, length_delete);
  recv_str.toCharArray(receivedCharsS, numChars);
  ptr = strtok(receivedCharsS, "<,>");
  int i = 0; 
  while (ptr != NULL){
    signals[i++] = (int)atoi(ptr);
    ptr = strtok(NULL, ","); 
  }
  for (int i = 0; i < NUM_CHANNELS; i++){
      signal_to_read[i] = (float)signals[i] / 
                            calibration_vector[i]; // TODO(sarahaguasvivas): normalize by calibration
  }
}

void receive_data() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    delay(2);
    char rc;
        while (Serial2.available() > 0 && newDataS == false) {
        rc = Serial2.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedCharsS[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedCharsS[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newDataS = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void toggle_data_flag() {
    if (newDataS == true) {
        newDataS = false;
    }
}