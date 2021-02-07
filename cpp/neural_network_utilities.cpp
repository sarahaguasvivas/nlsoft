#include "neural_network_utilities.hpp"

void nn_gradients(Matrix2 * first_derivative, Matrix2 * second_derivative, int n, int m, int nd, int input_size, float * input)
{

    float epsilon = 8e-3;
    int size_first = first_derivative->rows * first_derivative->cols;
    int size_second = second_derivative->rows* second_derivative->cols;

    set_to_zero(*first_derivative);
    set_to_zero(*second_derivative);

    float *output  = (float*)malloc(n*sizeof(float));
    float *output_minus_h = (float*)malloc(n*sizeof(float));
    float *output_plus_h = (float*)malloc(n*sizeof(float));       

    buildLayers();

    int nn = 0; 
    for (int j=0; j < m; j++)
    {
        for (int i = 0; i < n; i++) 
        {   
            float * input_center = (float*)malloc(input_size*sizeof(float));
            float * input_plus_h = (float*)malloc(input_size*sizeof(float));
            float * input_minus_h = (float*)malloc(input_size*sizeof(float));
            
            for (int i = 0; i<input_size;i++) input_center[i] = input[i];
            for (int i = 0; i<input_size;i++) input_plus_h[i] = input[i];
            for (int i = 0; i<input_size;i++) input_minus_h[i] = input[i];

            input_plus_h[nn] += epsilon;
            input_minus_h[nn] -= epsilon;
            
            output = fwdNN(input_center);
            output_plus_h = fwdNN(input_plus_h);
            output_minus_h = fwdNN(input_minus_h);

            first_derivative->data[i*first_derivative->cols + j] += (output_plus_h[i] - output_minus_h[i]) / (2. * epsilon);
            second_derivative->data[i*second_derivative->cols + j] += (output_plus_h[i] - 2.*output[i] + output_minus_h[i]) / (epsilon * epsilon);    
            
            nn++;
        }
    }

    free(output);
    free(output_minus_h);
    free(output_plus_h);
}