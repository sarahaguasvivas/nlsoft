#include <stdlib.h>

#define PI 3.1415926535897932384626433832795
#define NUM_SIGNAL  18
#define NN_INPUT_S   36

struct Controller{
  float machine_zero = 1e-20;
  float epsilon = 4e-3;
  float signal_calibration = 5e5;
  int m = 6, n = 3, nd = 2, dd = 2, N = 2, Nc = 1;
  float s = 1e-35, b = 1e-10, r = 4e5; 
  int nn_input_size = NN_INPUT_S;
  float neutral_point[3] = { 0., 0., 0.};
  int wavelength = 10;
  float medians_signals[NUM_SIGNAL]  = {
                                -0.5085999966, -0.4826000035, -0.6096000075, -0.5080999732, -0.5307999849,
                                -0.5483000278, -0.4607999921, -0.5889999866, -0.4124000072, -0.4099999964,
                                -0.4063999951, -0.4963999987,  0.0073999763, -0.0022000074, -0.1313000023,
                                 0.5650999546,  0.6205999851,  0.6116000414,        
                                        };
  float previous_input[NN_INPUT_S] = { 0.0 };
  float q_matrix[9] = { 1e6, 0., 0.,
                        0., 1e6,  0., 
                        0., 0., 2e6 };
  float lambda_matrix[36] = { 1., 0., 0., 0., 0., 0.,
                             0., 1., 0., 0., 0., 0., 
                             0., 0., 1., 0., 0., 0.,
                             0., 0., 0., 1., 0., 0.,
                             0., 0., 0., 0., 1., 0., 
                             0., 0., 0., 0., 0., 1.};
  float min_max_input_saturation[2] = { -0.5, 0.5 };
  float u[6] = { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5 };
  float prev_u[6] = { -0.5, -0.5, -0.5, -0.5, -0.5, -0.5 };
  float del_u[6] =  { 0., 0., 0., 0., 0., 0. };
};