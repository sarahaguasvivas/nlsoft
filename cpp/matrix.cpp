#include "matrix.hpp"
#include <math.h>
#include <stdlib.h>

void set(struct Matrix2 & a, int rows, int cols)
{
    a.rows = rows;
    a.cols = cols; 
    if (a.rows > 0 && a.cols > 0) a.data = (float*)malloc(a.rows*a.cols*sizeof(float));
}

void release(struct Matrix2 & a)
{
    free(a.data);
    a.data = NULL;
}

struct Matrix2 transpose(struct Matrix2 a)
{
    int size = a.rows*a.cols;
    
    Matrix2 transp;
    set(transp, a.rows, a.cols);

    if (a.rows == a.cols){
        for (int i = 0 ; i < size; i++) transp.data[i] = a.data[i];

        for (int i = 0 ; i< a.rows; i++)
        {
            for (int j = 0; j< a.cols; j++)
            {
                int forward_idx = i * a.cols + j;
                int transposed_idx = j * a.cols + i;
                transp.data[forward_idx] = a.data[transposed_idx];
            }
        }
        int temp = a.rows;
        transp.rows = a.cols;
        transp.cols = a.rows;
    } 
    return transp;
}

struct Matrix2 add (struct Matrix2 a, struct Matrix2 b)
{
    Matrix2 result;
    set(result, a.rows, a.cols);

    if (a.rows == b.rows && a.cols == b.cols)
    {
        for (int i=0; i< a.rows; i++)
        {
            for (int j=0; j< a.cols; j++)
            {
                int flat_idx = i*a.cols + j;
                result.data[flat_idx] = a.data[flat_idx] + b.data[flat_idx];
            }
        }
    }
    return result;
}

struct Matrix2 subtract (struct Matrix2 a, struct Matrix2 b)
{
    Matrix2 result;
    set(result, a.rows, a.cols);

    if (a.rows == b.rows && a.cols == b.cols)
    {
        for (int i=0; i< a.rows; i++)
        {
            for (int j=0; j< a.cols; j++)
            {
                int flat_idx = i*a.cols + j;
                result.data[flat_idx] = a.data[flat_idx] - b.data[flat_idx];
            }
        }
    }
    return result;
}

struct Matrix2 hadamard (struct Matrix2 a, struct Matrix2 b)
{
    Matrix2 result;
    set(result, a.rows, a.cols);

    if (a.rows == b.rows && a.cols == b.cols)
    {
        for (int i=0; i< a.rows; i++)
        {
            for (int j=0; j< a.cols; j++)
            {
                int flat_idx = i*a.cols + j;
                result.data[flat_idx] = a.data[flat_idx] * b.data[flat_idx];
            }
        }
    }
    return result;
}

void set_to_zero(struct Matrix2 & a){
    int size = a.rows*a.cols;
    for (int i = size - 1; i >= 0; i--) a.data[i] = 0.0;
}

struct Matrix2 multiply(struct Matrix2 a, struct Matrix2 b)
{
    Matrix2 product;
    set(product, a.rows, b.cols);
    
    if (a.cols == b.rows)
    {
        for (int i = 0; i < a.rows; ++i)
        {
            for (int j = 0; j < b.cols; ++j)
            {
                
                product.data[i*b.cols + j] = 0.0;
                
                for (int k = 0; k < a.cols; ++k)
                {
                    product.data[i*b.cols + j] +=  a.data[i*a.cols + k] * b.data[k * b.cols + j];
                }
            }
        }
    }
    return product;
}
struct Matrix2 inverse (struct Matrix2)
{

}