
/********************
    nn4mc.cpp

    Code generated using nn4mc.

    This file implements the nerual network and associated functions.

*/
#include "nn4mc.h"
#include <stdlib.h>
#include "dense.h"


struct Dense dense;
struct Dense dense_1;
struct Dense dense_2;


void buildLayers(){

    
        dense = build_layer_dense(&dense_W[0], dense_b, 36.0, 100, 0x06);

        dense_1 = build_layer_dense(&dense_1_W[0], dense_1_b, 100, 15, 0x06);

        dense_2 = build_layer_dense(&dense_2_W[0], dense_2_b, 15, 3, 0x07);


}


float * fwdNN(float* data, float * h_tm1, bool reset)
{
    
        data = fwd_dense(dense, data);

        data = fwd_dense(dense_1, data);

        data = fwd_dense(dense_2, data);


    return data;
}

