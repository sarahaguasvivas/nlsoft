#include <iostream>
#include <cstdlib>
#include "matrix.hpp"

void print_matrix(struct Matrix2 a)
{
    std::cout << std::endl;
    std::cout << "["; 
    for (int i=0; i<a.rows; i++)
    {
        std::cout << "["; 
        for (int j=0; j<a.cols; j++)
        {
            std::cout << "," << a.data[i*a.cols + j];
        }
        std::cout << "]," << std::endl;
    }
    std::cout << std::endl;

}
void fill_with_random(struct Matrix2 & a){
    time_t t;
    int size = a.rows*a.cols;

    srand((unsigned) time(&t));
    for (int i = size - 1; i >= 0; i--) a.data[i] = (float)(rand() / (float)RAND_MAX);
}

int main(){
    Matrix2 a;
    set(a, 4, 4);

    fill_with_random(a);

    // testing transpose
    std::cout << "Transpose:" << std::endl;
    print_matrix(a);
    Matrix2 b = transpose(a);
    print_matrix(b); 

    // testing_sum
    std::cout << "Sum: " << std::endl;
    Matrix2 c = add(a, b); 
    print_matrix(c);

    // testing_subtraction:
    std::cout << "Subtract: " << std::endl;
    Matrix2 d = subtract(c, a);
    print_matrix(d);

    // testing_hadamard:
    std::cout << "Hadamard: " << std::endl;
    Matrix2 e = hadamard(c, d);
    print_matrix(e);

    // testing_multiply:
    std::cout << "Product: " << std::endl;
    Matrix2 f = multiply(d, e);
    print_matrix(f);


    release(a);
    release(b);
    release(c);
    release(d);
    release(e);
    release(f);

    return 0;
}