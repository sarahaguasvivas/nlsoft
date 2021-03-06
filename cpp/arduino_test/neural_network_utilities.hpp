#ifndef __NNUTIL_H__
#define __NNUTIL_H__

//#include <Arduino.h>
#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif

#include "nn4mc.hpp"
#include "matrix.hpp"
#include "controller.hpp"

void setup_nn_utils();

void nn_gradients(Matrix2 *, Matrix2 *, int, int, int, int, float *, float);

void roll_window(int, int, int, float *);

Matrix2 nn_prediction(int, int, int, int, int, int, int, float*, float*);

void build_input_vector(float *, float * u, float * signal_, float * posish, int ndm, int ddn, int m, int n, int);

float partial_delta_u_partial_u(int, int);
float kronecker_delta(int, int);
Matrix2 get_jacobian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *, struct Controller);
Matrix2 get_hessian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *, struct Controller);
void solve(Matrix2, Matrix2, Matrix2 &);

#ifdef __cplusplus
}
#endif
#endif
