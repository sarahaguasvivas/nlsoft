#include <stdlib.h>

#define PI 3.1415926535897932384626433832795
#define NUM_SIGNAL  18
#define NN_INPUT_S   36

struct Controller{
  float machine_zero = 1e-20;
  float epsilon = 1./240.;
  float signal_calibration = 1e4;
  int m = 6, n = 3, nd = 2, dd = 2, N = 3, Nc = 1;
  float s = 1e-20, b = 1e5, r = 4e3; // doesn't do much
  int nn_input_size = NN_INPUT_S;
  float neutral_point[3] = { 0., 0., 0.};
  float input_calibration = 20;
  float input_offset = 0.5;
  int wavelength = 1;
  float q_matrix[3] = { 1e2, 3e2, 5e2 };
  float lambda_matrix[6] = { 5e-2, 5e-2, 5e-2, 5e-2, 5e-2, 5e-2 };
  float min_max_input_saturation[2] = { -0.5, 0.5 };
  float u[6] = { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5 };
  float prev_u[6] = { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5 };
  float del_u[6] =  { 0., 0., 0., 0., 0., 0. };
};