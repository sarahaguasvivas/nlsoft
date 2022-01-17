
/********************
    nn4mc.cpp

    Code generated using nn4mc.

    This file implements the nerual network and associated functions.

*/
#include "nn4mc.h"
#include "gru.h"
#include "dense.h"

struct GRU gru;
struct Dense dense;

void buildLayers(){
    
    gru = build_layer_gru(
                            &dense_W[0],
                            &gru_Wrec[0],
                            &dense_b[0],
                            0x08,
                            0x07,
                            1,
                            68,
                            1
    );

        dense = build_layer_dense(&dense_W[0], dense_b, 1, 3, 0x06);
}
float * fwdNN(float* data)
{
    data =  fwd_gru(gru, data);
    data = fwd_dense(dense, data);
    return data;
}