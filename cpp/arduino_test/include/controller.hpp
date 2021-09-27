#include <stdlib.h>

#define PI 3.1415926535897932384626433832795

struct Controller{
  float epsilon = 5e-2;
  float signal_calibration[11] = {613., 134., 104., 200., 128., 146., 183., 1., 2., 7., 100.};
  int m = 2, n = 3, nd = 3, dd = 3, N = 2, Nc = 1;
  float s = 1e-20, b = 1e-5, r = 4e3;
  int input_size = 26;
  float neutral_point[3] = {-0.06709795916817293, -0.047865542156502586, -0.016102764150255758};
  float q_matrix[3] = {1e-3, 1e3, 1e3};
  float lambda_matrix[2] = {1., 1.};
  float min_max_input_saturation[2] = {-1.74532925, 0.872665};
  float u[2*1] = {-1.22173048, -0.872664626};
  float normalized_u[2*1] = {-1.22173048/PI, -0.872664626/PI};
  float prev_u[2*1] = {-1.22173048, -0.872664626};
  float del_u[2*1] = {-1.22173048, -0.872664626};
  float y[3*2] = {-0.06709795916817293, -0.047865542156502586, -0.016102764150255758, 
                  -0.06709795916817293, -0.047865542156502586, -0.016102764150255758};
  float past_nn_input[26] = {-1.22173048/PI, -0.872664626/PI, 
                             -1.22173048/PI, -0.872664626/PI, 
                             -1.22173048/PI, -0.872664626/PI, 
                            -0.06709795916817293, -0.047865542156502586, -0.016102764150255758, 
                            -0.06709795916817293, -0.047865542156502586, -0.016102764150255758, 
                            -0.06709795916817293, -0.047865542156502586, -0.016102764150255758, 
                             4.36746058e-02, 0.00000000e+00, 2.91666667e-05, 5.38666667e-03,0.00000000e+00, 
                             5.56841553e-01, 5.16749545e-01, 0.00000000e+00, 
                             0.00000000e+00, 0.00000000e+00, 4.33333333e-06};
};

float deg2rad(float);
void clip_action(Matrix2 &);
