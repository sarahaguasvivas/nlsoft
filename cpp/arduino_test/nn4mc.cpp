
/********************
    nn4mc.cpp

    Code generated using nn4mc.

    This file implements the nerual network and associated functions.

*/

#include "nn4mc.hpp"
#include "dense.hpp"

struct Dense dense_19;
struct Dense dense_20;

void buildLayers(){

   
        dense_19 = build_layer_dense(&dense_19_W[0], dense_19_b, 26, 8, 0x06);

        dense_20 = build_layer_dense(&dense_20_W[0], dense_20_b, 8, 3, 0x07);
 
}


float * fwdNN(float* data)
{

   
        data = fwd_dense(dense_19, data);

        data = fwd_dense(dense_20, data);
 

    return data;
}
