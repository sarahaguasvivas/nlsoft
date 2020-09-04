#include "matrix.hpp"

Matrix2D::Matrix2D(uint_16 rows, uint_16 cols){
    this->rows = rows;
    this->cols = cols;
    this->elem = rows*cols;
    this->value = (float*)malloc(elem * sizeof(float));
}

float * Matrix2D::at(uint_16 i, uint_16 j){
   int flat_elem = i*this->cols + j; 
   return this->value[flat_elem];
}

Matrix2D::Matrix2D operator+(Matrix2D* B){

    if ((this->rows == B->rows) && (this->cols == B->cols)){
        Matrix2D sum(this->rows, this->cols);
        for (uint_t i = this->elem-1; i >= 0; --i){
            sum->value[i] = this->value[i] + B->value[i];
        }
        return sum; 
    } else{
        return NULL;
    } // no exception handling in Arduino
}

Matrix2D::Matrix2D operator-(Matrix2D* B){
    if((this->rows == B->rows) && (this->cols && B->cols)){
        Matrix2D diff(this->rows, this->cols);
        for(uint_t i = this->elem - 1; i >= 0; --i){
            diff->value[i] = this->value[i] - B->value[i];
        }
    }
}

Matrix2D::Matrix2D operator*(Matrix2D * B){
    if (this->cols == B->rows){
        Matrix2D prod(this->rows, B->cols);
        //TODO
    }

}