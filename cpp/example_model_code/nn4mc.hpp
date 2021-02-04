
/********************
    nn4mc.h

    Code generated using nn4mc.

    This file defines a a neural network and associated functions.

*/

#ifndef __NEURAL_NETWORK_H__
#define __NEURAL_NETWORK_H__

/*#if __cplusplus
extern "C" {
#endif
*/
#include "dense.hpp"
#include "parameters.hpp"
#include <stdlib.h>

void buildLayers();
float * fwdNN(float*);

#endif

