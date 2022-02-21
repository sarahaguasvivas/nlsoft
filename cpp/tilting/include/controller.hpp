#include <stdlib.h>

#define PI 3.1415926535897932384626433832795
#define NUM_SIGNAL  18
#define NN_INPUT_S   36

struct Controller{
  float machine_zero = 1e-10;
  float epsilon = 5e-2;
  float signal_calibration =  1e4;
  int m = 6, n = 3, nd = 2, dd = 2, N = 2, Nc = 1;
  float s = 1e-20, b = 1e-10, r = 4e3;
  int nn_input_size = NN_INPUT_S;
  float neutral_point[3] = { 0.0, 0.0, 0.0};
  float q_matrix[3] = { 1e7, 1e7, 1e7 };
  float lambda_matrix[6] = { 1e2, 1e2, 1e2, 1e4, 1e4, 1e4};
  float min_max_input_saturation[2] = {-0.5, 0.5};
  float u[6] = { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5 };
  float normalized_u[6] = { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5 };
  float prev_u[6] = { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5 };
  float del_u[6] =  { 0.0 };
  float past_nn_input[NN_INPUT_S] = {-5.00000000e-01, -5.00000000e-01, -5.00000000e-01, -5.00000000e-01,
                                    -5.00000000e-01, -5.00000000e-01, -5.00000000e-01, -5.00000000e-01,
                                    -5.00000000e-01, -5.00000000e-01, -5.00000000e-01, -5.00000000e-01,
                                     0.0, 0.0, 0.0,  0.0,
                                     0.0, 0.0,  0.00000000e+00,  0.00000000e+00,
                                     0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                     0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                     0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                     0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00};
};