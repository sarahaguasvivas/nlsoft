#include <iostream>
#include <cstdlib>
#include <chrono>
#include "utilities/neural_network_utilities.hpp"
#include "neural_network_utilities.hpp"
#include "matrix.hpp"
#include "figure_eight_target.hpp"
#define NUM_SIGNAL  11
#define PI 3.1415926535897932384626433832795
#define STEP_SIZE 0.05

float deg2rad(float);
void collect_signal(float *, float *);
void spin_figure_eight_target(int, int, int, int, Matrix2 *, float *);

int timestamp;
float posish[3];
unsigned long elapsed;

void build_input_vector(float * vector, float * u, float * signal_, float * posish, int ndm, int ddn, int m, int n)
{

    roll_window(0, ndm, m, vector);
    for (int i = 0; i < m; i++) vector[i] = u[i];
    roll_window(ndm, ndm + ddn, n, vector);
    for (int i = 0; i < n; i++) vector[i + ndm] = posish[i]; 
    
    for (int i = ndm + ddn; i < ndm + ddn + NUM_SIGNAL; i++) vector[i] = signal_[i];
}

int main() {
            float signal_calibration[NUM_SIGNAL] = {613., 134., 104., 200., 128., 146., 183., 1., 2., 7., 100.};
            int m = 2, n = 3, nd = 5, dd = 5, N = 5, Nc = 2;
            float s = 1e-20, b = 1e-5, r = 4e3;
            int input_size = 36;
            
            float ini_posish[n] = {0.05, 0.03, -0.06};
            for (int i=0; i< n; i++) posish[i] = ini_posish[i];
            float ini_motor[m] = {deg2rad(-70.), deg2rad(-50.)};
            float q_matrix[n*n] = {1e-3, 1e3, 1e3};
            float lambda_matrix[m*m] = {1., 1.};
            
            Matrix2 Q;
            Matrix2 Lambda;
            Matrix2 target;
            
            set(Q, n, n);
            set(Lambda, m, m);
            set(target, N, 3);
            
            for (int i = 0; i< n; i++) Q.data[i*Q.cols + i] = q_matrix[i];
            for (int i = 0; i< m; i++) Lambda.data[i*Lambda.cols + i] = lambda_matrix[i];
            
            float signal_[NUM_SIGNAL];
            float * nn_input = (float*)malloc((input_size)*sizeof(float));
            float * u = (float*)malloc((Nc*m)*sizeof(float));
            
            collect_signal(signal_, signal_calibration);
            build_input_vector(nn_input, u, signal_, posish, nd*m, dd*n, m, n);
            
            for (int i=0; i < Nc; i++) 
                for (int j=0; j< m; j++)
                    u[i*m + j] = ini_motor[j];
            
            spin_figure_eight_target(timestamp, 0, N, n, &target, ini_posish);
            Matrix2 prediction = nn_prediction(N, Nc, n, m, NUM_SIGNAL + nd*m + dd*n, nd, dd, nn_input, u);
            Matrix2 first_derivative;
            Matrix2 second_derivative;
            set(first_derivative, n, m);
            set(second_derivative, n, m);
            nn_gradients(&first_derivative, &second_derivative, n, m, nd, input_size, nn_input);

            for (int i = 0; i < n; i++) posish[i] = prediction.data[0*prediction.cols + i];
            
            // Getting Jacobian:
            Matrix2 jacobian;
            Matrix2 jacobian3;

            Matrix2 u_matrix;
            set(u_matrix, Nc, m);
            std::cout << u_matrix.data << std::endl;
            
            for (int i=0; i< Nc*m; i++) u_matrix.data[i] = u[i];
            Matrix2 temp;
            temp = subtract(target, prediction);
            jacobian3 = multiply(temp, Q);
            release(temp);
            temp =multiply(u_matrix, Lambda);
            Matrix2 temp1 = scale(2., temp);
            release(temp);
            jacobian = add(jacobian3, temp1);
            release(temp1);
            release(jacobian3);
            jacobian.rows = Nc;
            jacobian.cols = m;
            
            // Getting Hessian: 
            Matrix2 hessian2;
            Matrix2 hessian;
            
            temp = hadamard(first_derivative, first_derivative);
            temp1 = multiply(Q, temp);
            release(temp);
            hessian2 = scale(2., temp1); 
            release(temp1);

            release(first_derivative);
            temp = subtract(target, prediction);
            temp1 = transpose(temp);
            release(temp);
            temp = multiply(Q, second_derivative);
            Matrix2 temp2 = transpose(temp);
            release(temp);
            temp = multiply(temp2, temp1); 
            release(temp2);
            release(temp1);
            temp2= scale(2., temp);
            release(temp);
            hessian = subtract(hessian2, temp2);
            release(temp2);
            release(hessian2);
            release(target);
            release(Q);
            release(second_derivative);
            release(Lambda);
            
            for (int h = 0; h < Nc; h++)
            {
                for (int j = 0; j < m; j++)
                {
                for (int i= 0; i < m; i++)
                {
                    jacobian.data[i*jacobian.cols + j] += (-s / pow(u[j*m + i] + r / 2.0 - b, 3) + s / pow(r / 2.0 + b - u[j*m + i], 2));
                    hessian.data[h*hessian.cols + h] += (2.*s / pow(u[j*m + i] + r / 2.0 - b, 3) + 2.0* s / pow(r / 2.0 + b - u[j*m + i], 3.));
                }
                }
            }

            release(prediction);
            free(u);
            free(nn_input);

            release(u_matrix);
            u_matrix = solve_matrix_eqn(hessian, jacobian);

            std::cout << u_matrix.data << std::endl;
            std::cout << u_matrix.original_data_pointer << std::endl;
            release(u_matrix);

            timestamp++;
            //print_matrix(u_matrix);
            
            release(hessian);
            release(jacobian);
            
  return 0;
}

float deg2rad(float deg)
{
    return deg*PI/180.;
}

void collect_signal(float * signal_, float * signal_calibration)
{
    for (int i = 0; i < NUM_SIGNAL; i++) signal_[i] = 1.0;
}

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

void spin_figure_eight_target(int timestep, int n1, int n2, int dims, Matrix2 * target, float * center)
{
  float wavelength = 400;
  float a = 8./1000.;
  float b = 15./1000.;
  
  for (int i=n1; i<n2; i++)
  {
    float x =  0.001*sin((timestep+i) / (wavelength))* sin((timestep+i) / (wavelength));
    float y = a * sin((timestep + i) / wavelength);
    float z = b * sin((timestep + i) / wavelength) *  cos((timestep + i)/wavelength); 

    target->data[i*target->cols + 0] = x;
    target->data[i*target->cols + 1] = y;
    target->data[i*target->cols + 2] = z;
  }
}