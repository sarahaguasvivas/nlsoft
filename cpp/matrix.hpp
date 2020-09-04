#include <iostream>
#include <math.h>

/*****************************************
              2D Matrix Library
         Lightweight matrix library
******************************************/

class Matrix2D{
    int rows, cols, elem;
    float * value;
    public:
        Matrix2D(uint_16 rows, uint_16 cols);
        float* at(uint_16, uint_16); // at(i, j)
        Matrix2D operator+(Matrix2D *);
        Matrix2D operator-(Matrix2D *);
        Matrix2D operator=(Matrix2D *); 
        Matrix2D operator*(Matrix2D *);     
        Matrix2D operator!(Matrix2D *); //transpose
        void assign(Matrix2D *, uint_16, uint_16);
};

