#include "neural_network_utilities.hpp"
#include "matrix.hpp"
#include "motors.hpp"
#include "figure_eight_target.hpp"

#define NUM_SIGNAL  11
#define PI 3.1415926535897932384626433832795
#define STEP_SIZE 0.05

int timestamp;
float posish[3];

void setup() {
  Serial.begin(115200);
  timestamp = 0;
}

void print_matrix(Matrix2 matrix)
{
  for (int i=0; i< matrix.rows; i++){
    for (int j=0; j< matrix.cols; j++){
      Serial.print(matrix.data[i*matrix.rows + j]);
      Serial.print(",");
    }
    Serial.println();
  }
}

void build_input_vector(float * vector, float * u, float * signal_, float * posish, int ndm, int ddn)
{
    for (int i = 0; i< ndm; i++) vector[i] = u[i];
    for (int i = ndm; i < ndm + ddn; i++) vector[i] = posish[i];
    for (int i = ndm + ddn; i < ndm + ddn + NUM_SIGNAL; i++) vector[i] = signal_[i];
}

void loop() {
  
  float signal_calibration[NUM_SIGNAL] = {613., 134., 104., 200., 128., 146., 183., 1., 2., 7., 100.};
  int m = 2, n = 3, nd = 5, dd = 5, N = 5, Nc = 2;
  float s = 1e-20, b = 1e-5, r = 4e3;
  int input_size = m*nd + n*dd + NUM_SIGNAL;
  
  float ini_posish[n] = {0.05, 0.03, -0.06};
  for (int i=0; i< n; i++) posish[i] = ini_posish[i];
  float ini_motor[m] = {deg2rad(-70.), deg2rad(-50.)};
  float q_matrix[n*n] = {1e-3, 0., 0., 0., 1e3, 0., 0., 0., 1e3};
  float lambda_matrix[m*m] = {1., 0., 0., 1.};
  
  Matrix2 Q;
  Matrix2 Lambda;
  Matrix2 target;
  set(Q, n, n);
  set(Lambda, m, m);
  set(target, N, 3);
  
  for (int i = 0; i< n*n; i++) Q.data[i] = q_matrix[i];
  for (int i = 0; i< m*m; i++) Lambda.data[i] = lambda_matrix[i];
  
  float signal_[NUM_SIGNAL];
  float * nn_input = (float*)malloc((NUM_SIGNAL + nd*m + dd*n)*sizeof(float));
  float * u = (float*)malloc((Nc*m)*sizeof(float));
  
  collect_signal(signal_, signal_calibration);
  build_input_vector(nn_input, u, signal_, posish, nd*m, dd*n);
  
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
  /* 
   *  Optimization code here
   */
  
  step_motor(u, m);

  for (int i = 0; i < n; i++) posish[i] = prediction.data[0*prediction.cols + i];
  
  // Getting Jacobian:
  
  Matrix2 jacobian;
  
  Matrix2 u_matrix;
  set(u_matrix, Nc, m);
  set(jacobian, Nc, m);
 
  for (int i=0; i< Nc*m; i++) u_matrix.data[i] = u[i];

  jacobian = multiply(subtract(target, prediction), Q);
  Serial.println(jacobian.rows);
  Seria.println(jacobian.cols)
  jacobian = add(jacobian, scale(2., multiply(u_matrix, Lambda))); 
  
  for (int h = 0; h< Nc; h++)
  {
    for (int j = 0; j < Nc; j++)
    {
      for (int i= 0; i< m; i++)
        jacobian.data[i*jacobian.cols + j] += (-s / pow(u[j*m + i] + r / 2.0 - b, 3) + s / pow(r / 2.0 + b - u[j*m + i], 2));
    }
  }
  jacobian.cols = m;
  // Getting Hessian: 
  Matrix2 hessian;
  set(hessian, Nc, Nc);
  //hessian = scale(2., multiply(Q, hadamard(first_derivative, first_derivative)));
  hessian.data[0] = 1.0;
  hessian.data[1*hessian.cols+ 1] = 1.0;
  
  hessian = subtract(hessian, scale(2., multiply(transpose(multiply(Q, second_derivative)), transpose(subtract(target, prediction)))));
  //hessian = add(hessian, scale(2., Lambda));

  for (int h = 0; h < Nc; h++)
  {
    for (int j = 0; j < m; j++)
    {
      for (int i= 0; i < m; i++)
        hessian.data[h*hessian.cols + h] += (2.*s / pow(u[j*m + i] + r / 2.0 - b, 3) + 2.0* s / pow(r / 2.0 + b - u[j*m + i], 3.));
    }
  }
  Serial.println();
  print_matrix(jacobian);
  Serial.println(jacobian.rows);
  Serial.println(jacobian.cols);
  Serial.println("jacobian");
  //u_matrix = solve_matrix_eqn(hessian, jacobian);

  /*
   * end of optimization code
   */
  
  timestamp++;
  release(hessian);
  release(jacobian);
  release(u_matrix);
  release(first_derivative);
  release(second_derivative);
  release(prediction);
  free(u);
  free(nn_input);
  release(Lambda);
  release(Q);
  release(target);
}

float deg2rad(float deg)
{
    return deg*PI/180.;
}

void collect_signal(float * signal_, float * signal_calibration)
{
    for (int i = 0; i < NUM_SIGNAL; i++) signal_[i] = analogRead(i)/signal_calibration[i];
}

void print_array(float * arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++) {
      Serial.print(arr[i]);
      Serial.print(",");
    }
    Serial.println();
}
