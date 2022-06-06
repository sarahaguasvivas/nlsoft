
/********************
    nn4mc.cpp

    Code generated using nn4mc.

    This file implements the nerual network and associated functions.

*/
#include "nn4mc.h"
#include <stdlib.h>
#include "dense.h"


struct Dense dense_1;
struct Dense dense_2;
struct Dense dense_3;


void buildLayers(){

    
        dense_1 = build_layer_dense(&dense_1_W[0], dense_1_b, 36, 10, 0x06);

        dense_2 = build_layer_dense(&dense_2_W[0], dense_2_b, 10, 5, 0x06);

        dense_3 = build_layer_dense(&dense_3_W[0], dense_3_b, 5, 3, 0x07);


}


float * fwdNN(float* data)
{

    
        data = fwd_dense(dense_1, data);

        data = fwd_dense(dense_2, data);

        data = fwd_dense(dense_3, data);


    return data;
}

