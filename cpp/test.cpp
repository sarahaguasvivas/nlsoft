#include <iostream>
#include <cstdlib>
#include <chrono>
#include "utilities/neural_network_utilities.hpp"

void print_matrix (struct Matrix2 a)
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
    
    
    Matrix2 hessian;
    set(hessian, 10, 10);
    hessian = add(a, multiply(a, e));
    std::cout << "multiple operations" << std::endl;
    print_matrix(hessian);
    
    
    release(c);
    release(a);
    release(b);

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
    
    release(f);
    release(g);
    
    //testing_inverse for random matrix:
    for (int i = 0; i<e.rows; i++) e.data[i*e.cols + i] *= 100;
    std::cout << "A:" << std::endl;
    print_matrix(e);
    Matrix2 h;
    set(h, e.rows, e.cols); 
    start = std::chrono::high_resolution_clock::now();
    h = inverse(e);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "elapsed: " << duration.count() << std::endl;
    std::cout << "A_inv: " << std::endl;
    print_matrix(h);
    Matrix2 test_inverse;
    set(test_inverse, h.rows, h.cols);
    test_inverse = multiply(e, h);
    std::cout << "A*A_inv" << std::endl;
    print_matrix(test_inverse);

    release(test_inverse);

    Matrix2 first_derivative;
    Matrix2 second_derivative;
    
    set(first_derivative, 3, 2);
    set(second_derivative, 3, 2);

    set_to_ones(first_derivative);
    set_to_ones(second_derivative);
    
    release(d);
    release(e);
    release(h);
   
    float* input = (float*)malloc(36*sizeof(float));
    for (int i=0; i< 36; i++) input[i] = 1.0;
    start = std::chrono::high_resolution_clock::now();
    nn_gradients(&first_derivative, &second_derivative, 3, 2, 3, 36, input);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "elapsed: " << duration.count() << std::endl;
    std::cout << "First derivative: " << std::endl;
    print_matrix(first_derivative);
    std::cout << "Second derivative: " << std::endl;
    print_matrix(second_derivative);

    release(first_derivative);
    release(second_derivative);
    free(input);

    float * input1 = (float*)malloc(36*sizeof(float));
    for (int i=0; i< 36; i++) input1[i] = (float)i;
    float * u = (float*)malloc(5*2*sizeof(float));
    for (int i=0; i< 2*5; i++) u[i] = (float)(i+1)*100;    
    
    start = std::chrono::high_resolution_clock::now();
    Matrix2 y_out = nn_prediction(10, 3, 3, 2, 36, 5, 5, input1, u);
    free(u); 
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "elapsed: " << duration.count() << std::endl;
    std::cout << std::endl <<  "Prediction: " << std::endl;
    print_matrix(y_out);


    
    release(y_out);
    return 0;
}