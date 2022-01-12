#include "signals.hpp"

SensorChannel chnl_7(7, 2, 30);
SensorChannel chnl_8(8, 3, 29);
SensorChannel chnl_9(9, 4, 28);
SensorChannel chnl_10(10, 5, 27);
SensorChannel chnl_11(11, 6, 26);
SensorChannel chnl_12(12, 7, 25);

int signals[NUM_CHANNELS];

void setup_signal_collector(){
    chnl_7.begin();
    chnl_8.begin();
    chnl_9.begin();
    chnl_10.begin();
    chnl_11.begin();
    chnl_12.begin();
}

void collect_signal(float * signal_to_read, float* calibration_vector, 
                                            int signal_size)
{
    // channel driver

}
