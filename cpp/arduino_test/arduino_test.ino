#include "neural_network_utilities.hpp"
#include "matrix.hpp"
#include "motors.hpp"
#include "signals.hpp"
#include "figure_eight_target.hpp"

#define NUM_SIGNAL  11
#define PI 3.1415926535897932384626433832795

float deg2rad(float);
void clip_action(Matrix2 &);
void build_input_vector(float * vector, float * u, float * prediction, float * signal_, float * posish, int ndm, int ddn, int m, int n);
float partial_delta_u_partial_u(int j, int h);
float kronecker_delta(int h, int j);

unsigned long timestamp;
float posish[3] = {-0.06709795916817293, -0.047865542156502586, -0.016102764150255758} ;
unsigned long elapsed;

float u[2*1] = {deg2rad(-70), deg2rad(-50)};
float prev_u[2*1] = {deg2rad(-70), deg2rad(-50)};
float del_u[2*1] = {deg2rad(-70), deg2rad(-50)};
float y[3*2] = {-0.06709795916817293, -0.047865542156502586, -0.016102764150255758, -0.06709795916817293, -0.047865542156502586, -0.016102764150255758};
float past_nn_input[26];

float epsilon = 5e-2;
float signal_calibration[NUM_SIGNAL] = {613., 134., 104., 200., 128., 146., 183., 1., 2., 7., 100.};
int m = 2, n = 3, nd = 3, dd = 3, N = 2, Nc = 1;
float s = 1e-20, b = 1e-5, r = 4e3;
int input_size = 26;

float ini_posish[3] = {-0.06709795916817293, -0.047865542156502586, -0.016102764150255758};
float ini_motor[2] = {deg2rad(-70), deg2rad(-50)};
float q_matrix[3] = {1e-3, 1e3, 1e3};
float lambda_matrix[2] = {1., 1.};
float min_max_input_saturation[2] = {deg2rad(-100), deg2rad(50.)};

void setup() {
  setup_motor();
  setup_signal_collector();
  timestamp = 0;
  Serial.begin(115200);
  while(!Serial);
}

void loop() {
  
    //elapsed = millis();
  
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
    
      for (int i = 0; i < Nc; i++){
          for (int j = 0; j < m; j++){
              for (int k = 0; k < nd; k++){
                past_nn_input[i*k*input_size + j] = ini_motor[j];
              }
          }
      }
      collect_signal(&signal_[0], signal_calibration, NUM_SIGNAL);
      for (int k = 0; k < nd*m; k++){
          past_nn_input[k] = ini_posish[k%nd];
      }
      for (int k = nd*m; k < nd*m + dd*n; k++) {
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
    
    collect_signal(&signal_[0], signal_calibration, NUM_SIGNAL);
    
    if (timestamp > 0){
      build_input_vector(nn_input, u, y, signal_, posish, nd*m, dd*n, m, n);
    }
    
    for (int i = 0 ; i < input_size ; i++) past_nn_input[i] = nn_input[i];
    
    prediction = nn_prediction(N, Nc, n, m, NUM_SIGNAL + nd*m + dd*n, nd, dd, nn_input, u);
    
    for (int i = 0; i < n; i++) {
        posish[i] = prediction.data[0*n + i];
    }
    
    set(ynu, n, m);
    set(dynu_du, n, m);
    nn_gradients(&ynu, &dynu_du, n, m, nd, input_size, nn_input, epsilon);
    spin_figure_eight_target(timestamp, 0, N, n, &target, ini_posish);
    
    del_y = subtract(target, prediction);
 
    for (int i = 0; i < N*n; i++){
       y[i] = prediction.data[i];
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
    Matrix2 accum;

    set(accum, Nc, m);
    set_to_zero(accum);
    
    sub_sum = multiply(del_y, Q);
    temp = multiply(sub_sum, ynu);
    release(sub_sum);
    sub_sum = sum_axis(temp, 0);
    release(temp);
    temp = multiply(del_u_matrix, Lambda);
    release(del_u_matrix);
    temp1 = scale(2., temp);
    release(temp);
    
    for (int h = 0; h < Nc; h++){
      for (int j = 0; j < Nc; j++){
        Matrix2 accum1;
        Matrix2 accum2;
        accum1 = scale(partial_delta_u_partial_u(h, j), temp1);
        accum2 = add(accum, accum1);
        release(accum1);
        release(accum);
        set(accum, Nc, m);
        equal(accum, accum2);
        release(accum2);
      }
    }
    release(temp1);
    temp2 = repmat(sub_sum, Nc, 0);
    release(sub_sum);
   
    jacobian = add(accum,temp2);
    release(temp2);
    release(accum);
    
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

    Matrix2 second_y;
    Matrix2 second_y1;
    Matrix2 scale_1;
    Matrix2 mult;
    Matrix2 mult1;
    Matrix2 sum1;
    Matrix2 sum2;

    sum1 = sum_axis(hessian, 0);
    sum2 = sum_axis(sum1, 1);
    release(sum1);

    set(second_y, 1, m);
    set(second_y1, m, 1);
    
    for (int h = 0; h < Nc; h++)
    {
      for (int mm = 0; mm < Nc; mm++)
      {
        hessian1.data[h*Nc+mm] = sum2.data[0];
       
        for (int j = 0; j < m ; j++)
        { 
          second_y.data[j] = partial_delta_u_partial_u(j, mm);
          second_y1.data[j] = partial_delta_u_partial_u(j, h);
        }
       
        scale_1 = scale(2., Lambda);
        mult = multiply(second_y, second_y1);
        
        mult1 = multiply(scale_1, mult);
        Matrix2 multt;
        Matrix2 multtt;
        multt = sum_axis(mult1, 0);
        multtt = sum_axis(multt, 1);
        release(multt);
        release(scale_1);
        release(mult);
        hessian1.data[h*Nc + mm] += multtt.data[0];
        release(multtt);
      }
    }

    release(sum2);
    release(second_y);
    release(second_y1);
    release(mult1);
    release(hessian);
    
    for (int h = 0; h < Nc; h++)
    {
      for (int j = 0; j < m; j++)
      {
        for (int i = 0; i < m; i++)
        {
          jacobian.data[i*jacobian.cols + j] += -s / pow(u[j*m + i] + r / 2.0 - b, 2) +  s / pow(r / 2.0 + b - u[j*m + i], 2.0);
          hessian1.data[h*hessian1.cols + h] += 2.0* s / pow((u[j*m + i] + r / 2. - b), 3.0) + 2.0 * s / pow(r/2. + b - u[j*m + i], 3.0);
        }
      }
    }
    hessian1.data[0] = 1e-5;
    
    Matrix2 u_matrix; 
    Matrix2 inv;
    
    inv = inverse(hessian1);
    
    Matrix2 minusj;
    minusj = scale(-1., jacobian);
    del_u_matrix = multiply(inv, minusj); 
    
    release(inv);
    release(minusj);
    set(u_matrix, del_u_matrix.rows, del_u_matrix.cols);
   
    for (int i = 0; i < Nc*m; i++) { 
      u_matrix.data[i] = u[i] - del_u_matrix.data[i];
      prev_u[i] = u[i];
      u[i] = u_matrix.data[i];
      del_u[i] = 0.0; //del_u_matrix.data[i];
    }
    
    // Clipping action:
    //clip_action(u_matrix);


    delay(1);
    print_matrix(u_matrix);
    
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

float kronecker_delta(int h, int j){
  if (h == j){
    return 1.;
  } else{
    return 0.;
  }
}

float partial_delta_u_partial_u(int j, int h){
  return kronecker_delta(h, j) - kronecker_delta(h, j-1);
}

void build_input_vector(float * vector, float * u, float * prediction, float * signal_, float * posish, int ndm, int ddn, int m, int n)
{
    roll_window(0, ndm, m, vector);
    for (int i = 0; i < m; i++) vector[i] = u[i];
    roll_window(ndm + 1, ndm + ddn, n, vector);
    for (int i = 0; i < n; i++) vector[i + ndm] = posish[i]; 
    for (int i = ndm + ddn; i < input_size; i++) vector[i] = signal_[i - (ndm + ddn)];
}
