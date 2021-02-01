#ifndef __MATRIX_H__
#define __MATRIX_H__

struct Matrix2{
    int rows;
    int cols;
    float *data;
};

void set(struct Matrix2 &, int, int);
void set_to_zero(struct Matrix2 &);
struct Matrix2 transpose(struct Matrix2);
struct Matrix2 add (struct Matrix2, struct Matrix2);
struct Matrix2 subtract (struct Matrix2, struct Matrix2);
struct Matrix2 multiply(struct Matrix2, struct Matrix2);
struct Matrix2 hadamard(struct Matrix2, struct Matrix2);
struct Matrix2 inverse (struct Matrix2);
void release(struct Matrix2 & a);

#endif