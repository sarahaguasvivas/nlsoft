#include "signals.hpp"

const byte numChars = NUM_CHARS;
char receivedCharsS[numChars];
boolean newDataS = false;
int signals[NUM_CHANNELS];

void setup_signal_collector(){
  Serial2.begin(2000000, SERIAL_8N1, RXD2_S, TXD2_S);
}

void collect_signal(float * signal_to_read, float* calibration_vector, int signal_size)
{
  receive_data();
  toggle_data_flag();

  char *ptr = NULL;
  ptr = strtok(receivedCharsS, "<,>");
  int i = 0; 
  while (ptr != NULL){
    signals[i++] = (int)atoi(ptr);
    ptr = strtok(NULL, ","); 
  }
  for (int i = 0; i < NUM_CHANNELS; i++){
      signal_to_read[i] = (float)signals[i]/calibration_vector[i];
  }
}

void receive_data() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
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
    Serial.println(receivedCharsS);
}

void toggle_data_flag() {
    if (newDataS == true) {
        newDataS = false;
    }
}