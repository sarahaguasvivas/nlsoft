#include "neural_network_utilities.hpp"
#include "motors.hpp"

#define NUM_SIGNAL  11
#define PI 3.1415926535897932384626433832795

void setup() {
  Serial.begin(115200);
}

void loop() {
  int m = 2, n = 3, nd = 5, dd = 5;
  int N = 5, Nc = 5;
  float Q[n*n] = {1e-3, 0., 0., 0., 1e3, 0., 0., 0., 1e3};
  float Lambda[m*m] = {1., 0., 0., 1.};
  float step_size = 5e-2;
  
  float initial_position[n] = {0.05, 0.03, -0.06};
  float initial_input[m] = {deg2rad(-70.), deg2rad(-50.)};
  float signal_[NUM_SIGNAL];
  collect_signal(signal_);
  print_array(signal_, NUM_SIGNAL);
}

float deg2rad(float deg)
{
    return deg*PI/180.;
}

void collect_signal(float * signal_)
{
    for (int i = 0; i < NUM_SIGNAL; i++) signal_[i] = analogRead(i);
}

void print_array(float * arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++) {
      Serial.print(arr[i]);
      Serial.print(",");
    }
    Serial.println();
}
