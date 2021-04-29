#include "neural_network_utilities.hpp"
#include "matrix.hpp"
#include "motors.hpp"
#include "signals.hpp"
#include "figure_eight_target.hpp"

#define NUM_SIGNAL  11
#define PI 3.1415926535897932384626433832795
#define Ts 0.05

unsigned long timestamp;
float posish[3];
unsigned long elapsed;

float signal_calibration[NUM_SIGNAL] = {613., 134., 104., 200., 128., 146., 183., 1., 2., 7., 100.};
int m = 2, n = 3, nd = 5, dd = 5, N = 5, Nc = 2;
float s = 1e-20, b = 1e-5, r = 4e3;
int input_size = 36;
float ini_posish[3] = {0.05, 0.03, -0.06};
float ini_motor[2] = {0.0, 0.0};
float q_matrix[3] = {1e-3, 1e3, 1e3};
float lambda_matrix[2] = {1., 1.};
float signal_[NUM_SIGNAL];
Matrix2 Q;
Matrix2 Lambda;

void setup() {

  setup_motor();
  setup_signal_collector();
  timestamp = 0;

  set(Q, n, n);
  set(Lambda, m, m);
  set_to_zero(Q);
  set_to_zero(Lambda);
  for (int i = 0; i < n; i++) Q.data[i*n+i] = q_matrix[i];
  for (int i = 0; i < m; i++) Lambda.data[i*m + i] = lambda_matrix[i];
  Serial.begin(115200);
  while(!Serial);
}

void print_matrix(Matrix2 matrix)
{
  for (int i=0; i< matrix.rows; i++){
    for (int j=0; j< matrix.cols; j++){
      Serial.print(matrix.data[i*matrix.cols + j], 10);
      Serial.print(",");
    }
    Serial.println();
  }
  Serial.println();
}

void build_input_vector(float * vector, float * u, float * signal_, float * posish, int ndm, int ddn, int m, int n)
{
    roll_window(0, ndm, m, vector);
    for (int i = 0; i < m; i++) vector[i] = u[i];
    roll_window(ndm, ndm + ddn, n, vector);
    for (int i = 0; i < n; i++) vector[i + ndm] = posish[i]; 
    for (int i = ndm + ddn; i < ndm + ddn + NUM_SIGNAL; i++) vector[i] = signal_[i];
}

void loop() {
  
  elapsed = millis();
  Matrix2 del_y;
  Matrix2 u_matrix;
  Matrix2 del_u;
  Matrix2 target;
  set(del_y, N, n);
  set(u_matrix, Nc, m);
  set(del_u, Nc, m);
  set(target, N, n);

  float * nn_input = (float*)malloc((input_size)*sizeof(float));
  float * u = (float*)malloc((Nc*m)*sizeof(float));

  if (timestamp == 0){
    for (int i = 0; i < n; i++) posish[i] = ini_posish[i];
    for (int i = 0; i < Nc; i++){
        for (int j = 0; j < m; j++){
            u[i*m+j] = ini_motor[j];
            u_matrix.data[i*m+j] = ini_motor[j];
            del_u.data[i*m+j] = 0.0;
        }
    }
  }
  
  collect_signal(signal_, signal_calibration, NUM_SIGNAL);
  build_input_vector(nn_input, u, signal_, posish, nd*m, dd*n, m, n);
  Matrix2 prediction = nn_prediction(N, Nc, n, m, NUM_SIGNAL + nd*m + dd*n, nd, dd, nn_input, u);
  for (int i = 0; i < n; i++) posish[i] = prediction.data[(N-1)*prediction.cols + i];
  Matrix2 ynu;
  Matrix2 dynu_du;
  set(ynu, n, m);
  set(dynu_du, n, m);
  nn_gradients(&ynu, &dynu_du, n, m, nd, input_size, nn_input);

  spin_figure_eight_target(timestamp, 0, N, n, &target, ini_posish);
  
  del_y = subtract(target, prediction);
  release(prediction);
  release(target);
  //////////////////////////////////
  ///     Jacobian
  //////////////////////////////////
  Matrix2 jacobian;
  
  Matrix2 sub_sum;
  set(sub_sum, Nc, m);
  sub_sum = multiply(del_y, Q);
  Matrix2 temp;
  temp = multiply(sub_sum, ynu);
  release(sub_sum);
  sub_sum = sum_axis(temp, 0);
  release(temp);
  temp = multiply(del_u, Lambda);
  jacobian = scale(2., temp);
  release(temp);
  
  //////////////////////////////////
  ///     Hessian
  //////////////////////////////////
  Matrix2 hessian;
  temp = hadamard(ynu, ynu);
  Matrix2 temp1;
  temp1 = multiply(Q, temp);
  release(temp);
  temp = scale(2., temp1);
  release(temp1);
  temp1 = multiply(Q, dynu_du);
  Matrix2 temp2;
  temp2 = multiply(del_y, temp1);
  release(temp1);
  temp1 = scale(2., temp2);
  release(temp2);
  Matrix2 temp3;
  temp3 = sum_axis(temp, 0);
  release(temp);
  Matrix2 temp4;
  temp4 = sum_axis(temp1, 0);
  release(temp1);
  hessian = subtract(temp3, temp4);
  release(temp4);
  release(temp3);
  
  for (int h = 0; h < Nc; h++)
  {
    for (int j = 0; j < m; j++)
    {
      for (int i= 0; i < m; i++)
      {
        jacobian.data[i*jacobian.cols + j] += (-s / pow(u[j*m + i] + r / 2.0 - b, 3.) + s / pow(r / 2.0 + b - u[j*m + i], 2.));
        hessian.data[h*hessian.cols + h] += (2.*s / pow(u[j*m + i] + r / 2.0 - b, 3.) + 2.0* s / pow(r / 2.0 + b - u[j*m + i], 3.));
      }
    }
  }
  print_matrix(hessian);
 
  //u_matrix = solve_matrix_eqn(hessian, jacobian);
  //for(int i = 0; i < Nc*m; i++) u[i] = u_matrix.data[i];
  //step_motor(u_matrix.data, m);

  release(hessian);
  release(jacobian);
  release(ynu);
  release(dynu_du);
  release(u_matrix);
  release(del_y);
  free(nn_input);
  free(u);
  timestamp++;

  Serial.println(millis()-elapsed);
}

float deg2rad(float deg)
{
    return deg*PI/180.;
}

void print_array(float * arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++) {
      Serial.print(arr[i]);
      Serial.print(",");
    }
    //Serial.println();
}
