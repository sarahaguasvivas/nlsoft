#include <iostream>
#include <cstdlib>
#include "matrix.hpp"
#include <chrono>
#include "neural_network_utilities.hpp"

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
    set(a, 10, 10);

    fill_with_random(a);

    // testing transpose
    std::cout << "Transpose:" << std::endl;
    print_matrix(a);
    auto start = std::chrono::high_resolution_clock::now();
    Matrix2 b = transpose(a);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    print_matrix(b); 
    std::cout << "elapsed: " << duration.count() << std::endl;

    // testing_sum
    std::cout << "Sum: " << std::endl;
    start = std::chrono::high_resolution_clock::now();
    Matrix2 c = add(a, b); 
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    print_matrix(c);
    std::cout << "elapsed: " << duration.count() << std::endl;


    // testing_subtraction:
    std::cout << "Subtract: " << std::endl;
    start = std::chrono::high_resolution_clock::now();
    Matrix2 d = subtract(c, a);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    print_matrix(d);
    std::cout << "elapsed: " << duration.count() << std::endl;
    
    
    // testing_hadamard:
    std::cout << "Hadamard: " << std::endl;
    start = std::chrono::high_resolution_clock::now();
    Matrix2 e = hadamard(c, d);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "elapsed: " << duration.count() << std::endl;
    print_matrix(e);

    // testing_multiply:
    std::cout << "Product: " << std::endl;
    start = std::chrono::high_resolution_clock::now();
    Matrix2 f = multiply(d, e);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "elapsed: " << duration.count() << std::endl;
    print_matrix(f);
        
    
    // testing_inverse:
    std::cout << "Inverse: " << std::endl;
    set_to_zero(f);
    for (int i=0; i<f.rows; i++) f.data[i*a.cols+i] = 1.0;
    print_matrix(f);
    start = std::chrono::high_resolution_clock::now();
    Matrix2 g = inverse(f);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "elapsed: " << duration.count() << std::endl;
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