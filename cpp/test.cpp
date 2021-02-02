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

    // testing_inverse:
    std::cout << "Inverse: " << std::endl;
    set_to_zero(f);
    for (int i=0; i<f.rows; i++) f.data[i*a.cols+i] = 1.0;
    print_matrix(f);
    Matrix2 g = inverse(f);
    print_matrix(g);

    //testing_inverse for random matrix:
    for (int i = 0; i<e.rows; i++) e.data[i*e.cols + i] *= 100;
    std::cout << "A:" << std::endl;
    print_matrix(e);
    Matrix2 h;
    set(h, e.rows, e.cols); 
    h = inverse(e);
    std::cout << "A_inv: " << std::endl;
    print_matrix(h);
    Matrix2 test_inverse;
    set(test_inverse, h.rows, h.cols);
    test_inverse = multiply(e, h);
    std::cout << "A*A_inv" << std::endl;
    print_matrix(test_inverse);

    release(a);
    release(b);
    release(c);
    release(d);
    release(e);
    release(f);
    release(g);
    release(h);
    release(test_inverse);

    return 0;
}