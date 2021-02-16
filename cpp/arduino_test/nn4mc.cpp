
/********************
    nn4mc.cpp

    Code generated using nn4mc.

    This file implements the nerual network and associated functions.

*/

#include "nn4mc.hpp"
#include "dense.hpp"

struct Dense dense_9;
struct Dense dense_10;

void buildLayers()
{
   
        dense_9 = build_layer_dense(&dense_9_W[0], dense_9_b, 36, 15, 0x06);

        dense_10 = build_layer_dense(&dense_10_W[0], dense_10_b, 15, 3, 0x07);

}


float * fwdNN(float* data)
{
        data = fwd_dense(dense_9, data);

        data = fwd_dense(dense_10, data);
 
    return data;
}
