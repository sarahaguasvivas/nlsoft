
/********************
    nn4mc.cpp

    Code generated using nn4mc.

    This file implements the nerual network and associated functions.

*/
#include "nn4mc.h"
#include <stdlib.h>
#include "gru.h"
#include "dense.h"


struct GRU gru;
struct Dense dense;
struct Dense dense_1;


void buildLayers(){

    
gru = build_layer_gru(
                          &gru_W[0],
                          &gru_Wrec[0],
                          &gru_b[0],
                          0x08,
                          0x07,
                          1,
                          38,
                          10
);

        dense = build_layer_dense(&dense_W[0], dense_b, 10, 10, 0x06);

        dense_1 = build_layer_dense(&dense_1_W[0], dense_1_b, 10, 3, 0x07);


}


float * fwdNN(float* data, float* h_tm1)
{

    
        data =  fwd_gru(gru, data, h_tm1);

        data = fwd_dense(dense, data);

        data = fwd_dense(dense_1, data);


    return data;
}

