#include "neural_network_utilities.hpp"
#include <iostream>


void roll_window(int start, int finish, int buffer_size, float * array)
{
    for (int i = finish - buffer_size; i >= start; i--) 
    {
        array[i + buffer_size] = array[i];
    }
}

Matrix2 nn_prediction(int N, int Nc, int n, int m, int input_size, int nd, int dd, float * previous_input, float * u)
{
    Matrix2 y_output;
    set(y_output, N, n);
    buildLayers();

    for (int i = 0; i < N; i++)
    {
        float * input_next = (float*)malloc(input_size*sizeof(float));        
        for (int j = 0; j < input_size; j++) input_next[j] = previous_input[j];
        
        float * output_next;
        output_next = fwdNN(input_next);
        roll_window(0, nd*m - 1, m, previous_input);
        if (i < Nc)
        {
            for (int k = 0; k < m; k++)
            {
               previous_input[k] = u[i*m + k];
            }        
        } 
        else
        {
            for (int k = 0; k < m; k++)
            {
               previous_input[k] = u[(Nc-1)*m + k];
            }
        }
        roll_window(nd*m, nd*m + dd*n - 1, n, previous_input);
        for (int j = m*nd; j < m*nd + n; j++)
        { 
            previous_input[j] = output_next[j - m*nd];
        }
        for (int j = 0; j < n; j++)
        {
            y_output.data[i * n + j] = output_next[j];
        }
        free(output_next);
    }
    return y_output;
}


void nn_gradients(Matrix2 * first_derivative, Matrix2 * second_derivative, int n, int m, int nd, int input_size, float * input)
{
    float epsilon = 8e-3;
    int size_first = first_derivative->rows * first_derivative->cols;
    int size_second = second_derivative->rows* second_derivative->cols;

    set_to_zero(*first_derivative);
    set_to_zero(*second_derivative);

    
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
            
            float *output;
            float *output_minus_h;
            float *output_plus_h;       
              
            output = fwdNN(input_center);
            output_plus_h = fwdNN(input_plus_h);
            output_minus_h = fwdNN(input_minus_h);

            first_derivative->data[i*first_derivative->cols + j] += (output_plus_h[i] - output_minus_h[i]) / (2. * epsilon);
            second_derivative->data[i*second_derivative->cols + j] += (output_plus_h[i] - 2.*output[i] + output_minus_h[i]) / (epsilon * epsilon);    
            
            nn++;
            free(output);
            free(output_minus_h);
            free(output_plus_h);
        }
    }
}