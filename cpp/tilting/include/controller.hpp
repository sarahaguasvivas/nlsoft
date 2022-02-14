#include <stdlib.h>

#define PI 3.1415926535897932384626433832795
#define NUM_SIGNAL  18
#define NN_INPUT_S   36

struct Controller{
  float machine_zero = 1e-15;
  float epsilon = 5e-2;
  float signal_calibration[NUM_SIGNAL] = { 1e4 };
  int m = 6, n = 3, nd = 2, dd = 2, N = 2, Nc = 1;
  float s = 1e-10, b = 1e-15, r = 4e5;
  int nn_input_size = NN_INPUT_S;
  float neutral_point[3] = { 3.10689211e-06, -8.02683644e-08, -1.13993883e-06 };
  float q_matrix[3] = {1e3};
  float lambda_matrix[6] = {1.};
  float min_max_input_saturation[2] = {-0.5, 0.5};
  float u[6] = { -0.5 };
  float normalized_u[6] = { -0.5 };
  float prev_u[6] = { -0.5 };
  float del_u[6] =  { 0.0 };
  float past_nn_input[NN_INPUT_S] = {-5.00000000e-01, -5.00000000e-01, -5.00000000e-01, -5.00000000e-01,
                                      -5.00000000e-01, -5.00000000e-01, -5.00000000e-01, -5.00000000e-01,
                                      -5.00000000e-01, -5.00000000e-01, -5.00000000e-01, -5.00000000e-01,
                                        3.10689211e-06, -8.02683644e-08, -1.13993883e-06,  3.10689211e-06,
                                      -8.02683644e-08, -1.13993883e-06,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  2.09699988e-01,  2.53799975e-01,
                                        7.13999867e-02,  8.21200013e-01,  7.95899987e-01,  6.53200030e-01 };
};