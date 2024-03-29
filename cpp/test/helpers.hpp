#ifndef __HELPERS_H__
#define __HELPERS_H__

//#include <Arduino.h>
#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif

#include "nn4mc.hpp"
#include "matrix.hpp"
#include "controller.hpp"
#include "math.h"

#define PI 3.1415926535897932384626433832795

void setup_nn_utils();

float* single_prediction(float*);

void nn_gradients(Matrix2 *, Matrix2 *, int, int, int, int, float *, float);

void roll_window(int, int, int, float *);

void normalize_array(float*, float*, int, float);

Matrix2 nn_prediction(int, int, int, int, int,
                            int, int, float*, float*);

void build_input_vector(float *, float * u, float * signal_,
                            float * posish, int ndm, int ddn, int m, int n, int);

float partial_delta_u_partial_u(int, int);
float kronecker_delta(int, int);
Matrix2 get_jacobian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *, struct Controller);
Matrix2 get_hessian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *, struct Controller);
void solve(Matrix2, Matrix2, Matrix2 &);
void clip_action(Matrix2 &, Controller*);

#ifdef __cplusplus
}
#endif
#endif
