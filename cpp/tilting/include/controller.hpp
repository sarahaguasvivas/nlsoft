#include <stdlib.h>

#define PI 3.1415926535897932384626433832795

struct Controller{
  float machine_zero = 1e-10;
  float epsilon = 5e-2;
  float signal_calibration[18];
  int m = 6, n = 3, nd = 3, dd = 3, N = 1, Nc = 1;
  float s = 1e-15, b = 1e-5, r = 4e10;
  int nn_input_size = 68;
  float neutral_point[3] = { 0.};
  float q_matrix[3] = {1., 1e3, 1e3};
  float lambda_matrix[6] = {1.};
  float min_max_input_saturation[2] = {-1, 1};
  float u[6] = { 0.};
  float normalized_u[6] = { 0.};
  float prev_u[6] = { 0.};
  float del_u[6] =  { 0.};
  float y[3*1] = { 0.};
  float past_nn_input[68] = { 0. };
};

float deg2rad(float);
