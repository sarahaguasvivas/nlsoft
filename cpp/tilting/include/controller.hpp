#include <stdlib.h>

#define PI 3.1415926535897932384626433832795
#define NUM_SIGNAL  18
#define NN_INPUT_S   38

struct Controller{
  float machine_zero = 1e-10;
  float epsilon = 5e-2;
  float signal_calibration[NUM_SIGNAL] = {800., 800., 800., 800., 800., 800.};
  int m = 6, n = 3, nd = 2, dd = 2, N = 5, Nc = 1;
  float s = 1e-20, b = 1e-15, r = 4e5;
  int nn_input_size = NN_INPUT_S;
  float neutral_point[3] = { 0.};
  float q_matrix[3] = {1., 1.,  1.};
  float lambda_matrix[6] = {1.};
  float min_max_input_saturation[2] = {-1., 1.};
  float u[6] = { 0.};
  float normalized_u[6] = { 0.};
  float prev_u[6] = { 0.};
  float del_u[6] =  { 0.};
  float y[3] = { 0.};
  float past_nn_input[NN_INPUT_S] = { 0. };
};

float deg2rad(float);
