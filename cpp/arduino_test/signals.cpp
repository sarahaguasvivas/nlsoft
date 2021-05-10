#include "signals.hpp"

void setup_signal_collector(){
  //Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void collect_signal(float * signal_to_read, float* calibration_vector, int signal_size)
{
  //String to_read;
  //if (Serial1.available()){
    //to_read = String(Serial1.read());
    //sscanf(to_read.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", &signal_to_read[0], &signal_to_read[1], &signal_to_read[3], &signal_to_read[4], &signal_to_read[5], &signal_to_read[6], &signal_to_read[7], &signal_to_read[8], &signal_to_read[9], &signal_to_read[10]);
    for (int i = signal_size - 1 ; i>=0; i--) signal_to_read[i] /= calibration_vector[i];
    
  //}
}
