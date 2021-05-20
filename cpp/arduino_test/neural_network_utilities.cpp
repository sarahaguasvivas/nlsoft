#include "neural_network_utilities.hpp"

/*void roll_window(int buffer_size, float * vector)
{
    // offset by buffer size
    vector = vector - buffer_size;
}*/

void build_input_vector(float * vector, float * u,  float * signal_, float * posish, int ndm, int ddn, int m, int n)
{
    int input_size = ndm+ddn + (int)(sizeof(signal_)/sizeof(signal_[0]));
    roll_window(0, ndm, m, vector);
    for (int i = 0; i < m; i++) vector[i] = u[i];
    roll_window(ndm + 1, ndm + ddn, n, vector);
    for (int i = 0; i < n; i++) vector[i + ndm] = posish[i]; 
    for (int i = ndm + ddn; i < input_size; i++) vector[i] = signal_[i - (ndm + ddn)];
}

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
    float num_signal = input_size - nd*m - dd*n;
    float * signals = (float*)malloc(num_signal*sizeof(float));
    for (int i = 0; i < num_signal; i++) signals[i] = previous_input[i + nd*m + dd*n];
    float motor_input[m*nd];
    
    buildLayers();

    for (int i = 0; i < N; i++)
    {
        float * input_next = (float*)malloc(input_size*sizeof(float));        
        for (int j = 0; j < input_size; j++) input_next[j] = previous_input[j];
        
        float * output_next;
        
        output_next = fwdNN(input_next);

        if (i < Nc)
        {
            for (int k = 0; k < m; k++)
            {
               motor_input[k] = u[i*m + k];
            }        
        } 
        else
        {
            for (int k = 0; k < m; k++)
            {
               motor_input[k] = u[(Nc-1)*m + k];
            }
        }
        
        build_input_vector(previous_input, motor_input, signals, output_next, nd*m, dd*n, m, n);
        
        for (int j = 0; j < n; j++)
        {
            y_output.data[i * n + j] = output_next[j];
        }
        free(output_next);
    }
    free(signals);
    
    return y_output;
}


void nn_gradients(Matrix2 * first_derivative, Matrix2 * second_derivative, int n, int m, int nd, int input_size, float * input, float epsilon)
{
    int size_first = first_derivative->rows * first_derivative->cols;
    int size_second = second_derivative->rows* second_derivative->cols;

    set_to_zero(*first_derivative);
    set_to_zero(*second_derivative);

    buildLayers();
    
    float * output;
    float * output_minus_h; 
    float * output_plus_h;
    
    for (int nn = 0; nn < nd*m; nn++)
        {    
            int j = nn % m;
            float * input_center = (float*)malloc(input_size*sizeof(float));
            float * input_plus_h = (float*)malloc(input_size*sizeof(float));
            float * input_minus_h= (float*)malloc(input_size*sizeof(float));

            for (int i = 0; i<input_size;i++) input_center[i] = input[i];
            for (int i = 0; i<input_size;i++) input_plus_h[i] = input[i];
            for (int i = 0; i<input_size;i++) input_minus_h[i] = input[i];

            input_plus_h[nn] += epsilon;
            input_minus_h[nn] -= epsilon;
                  
            output = fwdNN(input_center);
            output_plus_h = fwdNN(input_plus_h);
            output_minus_h = fwdNN(input_minus_h);
            
            for (int i = 0 ; i < n; i++){
              first_derivative->data[i*first_derivative->cols + j] += (output_plus_h[i] - output_minus_h[i]) / (2. * epsilon);
              second_derivative->data[i*second_derivative->cols + j] += (output_plus_h[i] - 2.*output[i] + output_minus_h[i]) / (epsilon * epsilon);    
            }
            
            free(output);
            free(output_minus_h);
            free(output_plus_h);
        }
}
