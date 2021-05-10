#ifndef __NNUTIL_H__
#define __NNUTIL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "nn4mc.hpp"
#include "matrix.hpp"


void nn_gradients(Matrix2 *, Matrix2 *, int, int, int, int, float *, float);

void roll_window(int, int, int, float *);

Matrix2 nn_prediction(int, int, int, int, int, int, int, float*, float*);

#ifdef __cplusplus
}
#endif
#endif
