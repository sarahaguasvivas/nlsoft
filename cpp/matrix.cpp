#include "matrix.hpp"
#include <math.h>
#include <stdlib.h>
#include <iostream>

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
float * vector(int nl, int nh){
    float *v;
    v = (float*)malloc((size_t)((nh-nl+1+1)*sizeof(float)));
    return v-nl+1;
}

void free_vector(float *v, int nl, int nh){
    free((float*)(v+nl-1));
}

void ludcmp(struct Matrix2 a, int *indx, float * d)
{   /*
    References
        ----------
        [2] W. H. Press, S. A. Teukolsky, W. T. Vetterling and B. P. Flannery,
            "Numerical Recipes (3rd edition)", Cambridge University Press, 2007,
            page 46. https://www.cec.uchile.cl/cinetica/pcordero/MC_libros/NumericalRecipesinC.pdf
    */

   int imax;
   int n = a.rows;
   float big, dum, sum, temp;
   float * vv;
   int error = 0;

   vv = vector(1, n);
   *d = 1.0;
   
    for (int i = 1; i<=n; i++)
   {
       big = 0.0;
       for (int j=1; j <= n; j++)
       {
           if ((temp == fabs(a.data[i * a.cols + j])) > big) big = temp;
       }
       if (big == 0.0) {
           error = 1;
           vv[i] = 0.0;
       } else vv[i] = 1.0/big;
   }
      
    for (int j=1 ; j <= n; j++)
    {
        for (int i = 1; i< j; i++)
        {
            sum = a.data[i*a.cols + j];
            for (int k = 1; k < i; k++)
            {
                sum -= a.data[i*a.cols + k] * a.data[k*a.cols + j];
            }
            a.data[i*a.cols + j] = sum;
        }
        big = 0.0;
        for (int i = 1; i <=n; i++)
        {
            sum = a.data[i*a.cols + j];
            for (int k = 1; k<j; k++)
                sum -= a.data[i*a.cols + k] * a.data[k*a.cols + j];
            a.data[i*a.cols + j] = sum;
            if ((dum = vv[i]*fabs(sum)) >= big)
            {
                big = dum;
                imax = i;
            }
        }
                     
        if (j != imax){
            for (int k=1; k<=n; k++)
            {
                dum = a.data[imax*a.cols + k];
                a.data[imax * a.cols + k]= a.data[j*a.cols + k];
                a.data[j*a.cols + k] = dum;
            }
            *d = -(*d);
            vv[imax] = vv[j];
        }
             
        indx[j] = imax;
        if (a.data[j*a.cols + j] == 0.0) a.data[j*a.cols + j] = a.tiny;
        if (j!= n)
        {
            dum = 1.0/a.data[j*a.cols+j];
            for (int i = j+1; i <= n; i++) a.data[i*a.cols + j] *= dum;
        }
    }
  
    free_vector(vv, 1, n);
}

void lubksb(struct Matrix2 a, int *indx, float * b)
{
    int i, ii=0, ip, j;
    float sum;
    int n = a.cols;

    for (i = 1; i <= n; i++)
    {
        ip = indx[i];
        sum = b[ip];
        b[ip]= b[i];
        if (ii)
            for (j = ii; j<= i-1; j++) sum -= a.data[i*a.cols + j] * b[j];
        else if (sum) ii = i;
        b[i] = sum;
    }
    for (i = n; i >= 1; i--){
        sum = b[i];
        for (j = i+1; j<= n ; j++) sum -= a.data[i*a.cols+j] * b[j];
        b[i] = sum/a.data[i*a.cols + j];
    }
}

struct Matrix2 inverse (struct Matrix2 a)
{
    /*
        References
            ----------
            .. [1] MATLAB reference documention, "Rank"
                https://www.mathworks.com/help/techdoc/ref/rank.html
            .. [2] W. H. Press, S. A. Teukolsky, W. T. Vetterling and B. P. Flannery,
                "Numerical Recipes (3rd edition)", Cambridge University Press, 2007,
                page 795.
    */

    int i, j, *indx;
    float d, *col;
    
    indx = (int*)malloc(a.cols * sizeof(int));
    col = (float*)malloc(a.cols*sizeof(float));

    Matrix2 inverse;
    set(inverse, a.rows, a.cols);
    
    ludcmp(a, indx, &d);
    
    for (j = 1; j <= a.cols; j++)
    {
        for (i = 1; i <= a.cols; i++) col[i] = 0.0;
        col[j] = 1.0;
        lubksb(a, indx, col);
        for (i = 1; i <= a.cols; i++) inverse.data[i*a.cols + j] = col[i]; 
    }

    return inverse;
    free(indx);
    free(col);
}