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

float u[2*2];
float prev_u[2*2];
float del_u[2*2];
float y[5*3];
float past_nn_input[36];

float signal_calibration[NUM_SIGNAL] = {613., 134., 104., 200., 128., 146., 183., 1., 2., 7., 100.};
int m = 2, n = 3, nd = 5, dd = 5, N = 5, Nc = 2;
float s = 1e-20, b = 1e-5, r = 4e3;
int input_size = 36;
float ini_posish[3] = {0.05, 0.03, -0.06};
float ini_motor[2] = {-1.22173048, -0.872664626};
float q_matrix[3] = {1e-3, 1e3, 1e3};
float lambda_matrix[2] = {1., 1.};
float min_max_input_saturation[2] = {-1.7453, 0.873};

void setup() {
  setup_motor();
  setup_signal_collector();
  timestamp = 0;
  
  Serial.begin(115200);
  while(!Serial);
}

void clip_action(Matrix2 &u_matrix){
  for (int i = 0; i < Nc*m; i++){
    u_matrix.data[i] = min(u_matrix.data[i], min_max_input_saturation[1]); // max
    u_matrix.data[i] = max(u_matrix.data[i], min_max_input_saturation[0]); // min
  }
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
  //Serial.println();
}

void build_input_vector(float * vector, float * u, float * prediction, float * signal_, float * posish, int ndm, int ddn, int m, int n)
{
    roll_window(0, ndm, m, vector);
    for (int i = 0; i < m; i++) vector[i] = u[i];
    roll_window(ndm, ndm + ddn, n, vector);
    for (int i = 0; i < n; i++) vector[i + ndm] = posish[i]; 
    for (int i = ndm + ddn; i < input_size; i++) vector[i] = signal_[i - (ndm + ddn)];
}

void loop() {
  
  elapsed = millis();

    float * signal_ = (float*)malloc(NUM_SIGNAL*sizeof(float));
    Matrix2 Q;
    Matrix2 Lambda;
    Matrix2 del_u_matrix;
    Matrix2 del_y;
    Matrix2 target;
    Matrix2 prediction;
    Matrix2 ynu;
    Matrix2 dynu_du;
    
    set(Q, n, n);
    set(Lambda, m, m);
    set(del_u_matrix, Nc, m);
    set_to_zero(Q);
    set_to_zero(Lambda);
    
    for (int i = 0; i < n; i++) Q.data[i*n+i] = q_matrix[i];
    for (int i = 0; i < m; i++) Lambda.data[i*m + i] = lambda_matrix[i];
    set(target, N, n);

    float * nn_input = (float*)malloc((input_size)*sizeof(float));
    
    if (timestamp == 0) {
      set_to_zero(del_u_matrix);
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < N; j++){
          posish[i] = ini_posish[i];
          y[j*n+i] = ini_posish[i];
        }
      }
      for (int i = 0; i < Nc; i++){
          for (int j = 0; j < m; j++){
              u[i*m+j] = ini_motor[j];
              prev_u[i*m + j] = ini_motor[j];
              for (int k = 0; k < nd; k++){
                past_nn_input[i*k*input_size + j] = ini_motor[j];
              }
          }
      }
      collect_signal(&signal_[0], signal_calibration, NUM_SIGNAL);
      for (int k = 0; k < nd*m; k++){
          past_nn_input[k] = ini_posish[k%nd];
      }
      for (int k = nd*m; k < nd*m + dd*n; k++){
          past_nn_input[k] = ini_posish[(k - nd*m)%dd];
      }
      for (int k = nd*m + dd*n; k < input_size; k++){
          past_nn_input[k] = signal_[k - (nd*m + dd*n)];
      }
     
    } else{
      for (int i = 0; i < Nc; i++){
        for (int j = 0; j < m; j++){
          del_u_matrix.data[i*m+j] = del_u[i*m+j];
        }
      }
    }
    
    for (int i = 0; i < input_size; i++) {
      nn_input[i] = past_nn_input[i];
    }

    print_array(nn_input, input_size);
    
    collect_signal(&signal_[0], signal_calibration, NUM_SIGNAL);
    
    if (timestamp > 0){
      build_input_vector(nn_input, u, y, signal_, posish, nd*m, dd*n, m, n);
    }
    
    for (int i = 0 ; i<input_size ; i++) past_nn_input[i] = nn_input[i];
    
    prediction = nn_prediction(N, Nc, n, m, NUM_SIGNAL + nd*m + dd*n, nd, dd, nn_input, u);
    
    for (int i = 0; i < n; i++) {
        posish[i] = prediction.data[i];
    }
    
    set(ynu, n, m);
    set(dynu_du, n, m);
    
    nn_gradients(&ynu, &dynu_du, n, m, nd, input_size, nn_input, 5e-2);
    spin_figure_eight_target(timestamp, 0, N, n, &target, ini_posish);
    del_y = subtract(target, prediction);
 
    for (int i = 0; i < N; i++){
        for (int j = 0; j < n; j++){
          y[i*n+j] = prediction.data[i*n+j];
        }
    }
    release(prediction);
    release(target);
    
    //////////////////////////////////
    ///     Jacobian
    //////////////////////////////////
    Matrix2 jacobian;
    Matrix2 sub_sum;
    Matrix2 temp;
    Matrix2 temp1;
    Matrix2 temp2;

    sub_sum = multiply(del_y, Q);
    temp = multiply(sub_sum, ynu);
    release(sub_sum);
    sub_sum = sum_axis(temp, 0);
    release(temp);
    temp = multiply(del_u_matrix, Lambda);
    release(del_u_matrix);
    temp1 = scale(2., temp);
    release(temp);
    temp2 = repmat(sub_sum, Nc, 0);
    release(sub_sum);
    jacobian = add(temp1,temp2);
    release(temp1);
    release(temp2);
    
    //////////////////////////////////
    ///     Hessian
    //////////////////////////////////
    Matrix2 hessian;
    Matrix2 temp3;
    Matrix2 temp4;
    Matrix2 hessian1;
    
    temp = hadamard(ynu, ynu);
    release(ynu);
    temp1 = multiply(Q, temp);
    release(temp);
    temp = scale(2., temp1);
    release(temp1);
    temp1 = multiply(Q, dynu_du);
    release(dynu_du);
    temp2 = multiply(del_y, temp1);
    release(temp1);
    release(del_y);
    temp1 = scale(2., temp2);
    release(temp2);
    temp3 = sum_axis(temp, 0);
    release(temp);
    temp4 = sum_axis(temp1, 0);
    release(temp1);
    hessian = subtract(temp3, temp4);
    release(temp4);
    release(temp3);
    
    set(hessian1, Nc, Nc);
    set_to_zero(hessian1);
  
    for (int i = 0; i < m; i++){
      for (int j = 0; j < Nc; j++){
        hessian1.data[j*Nc+j] += hessian.data[i];
      }
    }
    release(hessian);
    
    for (int h = 0; h < Nc; h++)
    {
      for (int j = 0; j < m; j++)
      {
        for (int i = 0; i < m; i++)
        {
          jacobian.data[i*jacobian.cols + j] += (-s / pow(u[j*m + i] + r / 2.0 - b, 3.) + s / pow(r / 2.0 + b - u[j*m + i], 2.));
          hessian1.data[h*hessian1.cols + h] += (2.*s / pow(u[j*m + i] + r / 2.0 - b, 3.) + 2.0 * s / pow(r / 2.0 + b - u[j*m + i], 3.));
        }
      }
    }
    Matrix2 u_matrix; 
    Matrix2 inv;
    inv = inverse(hessian1);
    
    //u_matrix = solve_matrix_eqn(hessian1, jacobian);
    u_matrix = multiply(inv, jacobian);
    release(inv);

    // Clipping action:
    clip_action(u_matrix);

    for (int i = 0; i < Nc*m; i++) { 
      prev_u[i] = (float)u[i];
      u[i] = u_matrix.data[i];
      del_u[i] = (float)u_matrix.data[i] - (float)prev_u[i];
    }
    
    delay(1);
    //print_matrix(u_matrix);
    
    //step_motor(u_matrix.data, m);
    
    release(hessian1);
    release(jacobian);

    release(Q);
    release(Lambda);
    release(u_matrix);
    
    free(nn_input);
    free(signal_);
    timestamp++;

  //Serial.println(millis()-elapsed);
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
    Serial.println();
}
