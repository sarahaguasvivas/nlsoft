
/********************
    nn4mc.cpp

    Code generated using nn4mc.

    This file implements the nerual network and associated functions.

*/
#include "nn4mc.h"
#include <stdlib.h>

void buildLayers(){

   
        dense_19 = buildDense(&dense_19_W[0], dense_19_b, 26, 8, 0x06, 1);

        dense_20 = buildDense(&dense_20_W[0], dense_20_b, 8, 3, 0x07, 1);

}


float * fwdNN(float* data)
{
      
        data = fwdDense(dense_19, data);

        data = fwdDense(dense_20, data);
 

    return data;
}
