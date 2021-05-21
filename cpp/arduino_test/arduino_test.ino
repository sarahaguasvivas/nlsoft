#include "neural_network_utilities.hpp"
#include "matrix.hpp"
#include "motors.hpp"
#include "signals.hpp"
#include "figure_eight_target.hpp"

#define NUM_SIGNAL  11
#define PI 3.1415926535897932384626433832795

float deg2rad(float);
void clip_action(Matrix2 &);
float partial_delta_u_partial_u(int, int);
float kronecker_delta(int, int);
Matrix2 get_jacobian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *);
Matrix2 get_hessian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *);
void solve(Matrix2, Matrix2, Matrix2 &);

unsigned long timestamp;
float posish[3] = {-0.06709795916817293, -0.047865542156502586, -0.016102764150255758};
unsigned long elapsed;

struct Controller{
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
  float u[2*1] = {deg2rad(-70), deg2rad(-50)};
  float prev_u[2*1] = {deg2rad(-70), deg2rad(-50)};
  float del_u[2*1] = {deg2rad(-70), deg2rad(-50)};
  float y[3*2] = {-0.06709795916817293, -0.047865542156502586, -0.016102764150255758, -0.06709795916817293, -0.047865542156502586, -0.016102764150255758};
  float past_nn_input[26] = {deg2rad(-70), deg2rad(-50), deg2rad(-70), deg2rad(-50), deg2rad(-70), deg2rad(-50), -0.06709795916817293, -0.047865542156502586, -0.016102764150255758, -0.06709795916817293, -0.047865542156502586, -0.016102764150255758, -0.06709795916817293, -0.047865542156502586, -0.016102764150255758, 0, 0., 0,0,0,0,0,0,0,0,0};
};

Controller controller;

void setup() {
  setup_motor();
  setup_signal_collector();
  timestamp = 0;
  Serial.begin(115200);
  while(!Serial);
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
    
    set(Q, controller.n, controller.n);
    set(Lambda, controller.m, controller.m);
    set(del_u_matrix, controller.Nc, controller.m);
    set_to_zero(Q);
    set_to_zero(Lambda);
    
    for (int i = 0; i < controller.n; i++) Q.data[i*controller.n+i] = controller.q_matrix[i];
    for (int i = 0; i < controller.m; i++) Lambda.data[i*controller.m + i] = controller.lambda_matrix[i];
    set(target, controller.N, controller.n);

    float * nn_input = (float*)malloc((controller.input_size)*sizeof(float));
    if (timestamp == 0) {
      set_to_zero(del_u_matrix);
    } else{
      for (int i = 0; i < controller.Nc; i++){
        for (int j = 0; j < controller.m; j++){
          del_u_matrix.data[i*controller.m+j] = controller.del_u[i*controller.m+j];
        }
      }
    }
    
    for (int i = 0; i < controller.input_size; i++) {
      nn_input[i] = controller.past_nn_input[i];
    }
    
    collect_signal(&signal_[0], controller.signal_calibration, NUM_SIGNAL);
    
    if (timestamp > 0){
      build_input_vector(nn_input, controller.u, signal_, posish, controller.nd*controller.m, controller.dd*controller.n, controller.m, controller.n);
    }
    
    for (int i = 0 ; i < controller.input_size ; i++) controller.past_nn_input[i] = nn_input[i];
   
    prediction = nn_prediction(controller.N, controller.Nc, controller.n, controller.m, NUM_SIGNAL + controller.nd*controller.m + controller.dd*controller.n, controller.nd, controller.dd, nn_input, controller.u);
    
    for (int i = 0; i < controller.n; i++) {
        posish[i] = prediction.data[(controller.N-1)*controller.n + i];
    }
    
    set(ynu, controller.n, controller.m);
    set(dynu_du, controller.n, controller.m);
    nn_gradients(&ynu, &dynu_du, controller.n, controller.m, controller.nd, controller.input_size, nn_input, controller.epsilon);
    spin_figure_eight_target(timestamp, 0, controller.N, controller.n, &target, controller.ini_posish);

    del_y = subtract(target, prediction);
  
    for (int i = 0; i < controller.N*controller.n; i++){
       controller.y[i] = prediction.data[i];
    }
    release(prediction);
    release(target);
    
    // jacobian ////////////////////////////////////////////////////////////////////
    Matrix2 jacobian;
    jacobian = get_jacobian(del_y, Q, Lambda, ynu, dynu_du, del_u_matrix, controller.u, controller.del_u);
    
    // hessian /////////////////////////////////////////////////////////////////////
    Matrix2 hessian;
    hessian = get_hessian(del_y, Q, Lambda, ynu, dynu_du, del_u_matrix, controller.u, controller.del_u);
    
    ////////////////////////////////////////////////////////////////////////////////
    release(del_y);
    
    Matrix2 u_matrix; 
    
    solve(jacobian, hessian, del_u_matrix);
    set(u_matrix, del_u_matrix.rows, del_u_matrix.cols);
   
    for (int i = 0; i < controller.Nc*controller.m; i++) { 
      u_matrix.data[i] = controller.u[i] - del_u_matrix.data[i];
    }
   
    // Clipping action:
    //clip_action(u_matrix);

    delay(5);
    print_matrix(u_matrix);

    for (int i = 0; i < controller.Nc*controller.m; i++) { 
      controller.prev_u[i] = controller.u[i];
      controller.u[i] = u_matrix.data[i];
      controller.del_u[i] = del_u_matrix.data[i];
    }
    
    //step_motor(u_matrix.data, m);

    release(hessian);
    release(jacobian);
    release(Q);
    release(Lambda);
    release(u_matrix);
    release(del_u_matrix);
    
    free(nn_input);
    free(signal_);
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
      Serial.print(arr[i], 10);
      Serial.print(",");
    }
    Serial.println();
}

void clip_action(Matrix2 &u_matrix){
  for (int i = 0; i < controller.Nc*controller.m; i++){
    u_matrix.data[i] = min(u_matrix.data[i], controller.min_max_input_saturation[1]); // max
    u_matrix.data[i] = max(u_matrix.data[i], controller.min_max_input_saturation[0]); // min
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

Matrix2 get_jacobian(Matrix2 del_y, Matrix2 Q, Matrix2 Lambda, Matrix2 ynu, Matrix2 dynu_du, Matrix2 del_u_matrix, float * u, float * del_u){
     
    Matrix2 jacobian;
    Matrix2 sub_sum;
    Matrix2 temp;
    Matrix2 temp1;
    Matrix2 temp2;
    Matrix2 accum;

    set(accum, controller.Nc, controller.m);
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
    
    for (int h = 0; h < controller.Nc; h++){
      for (int j = 0; j < controller.Nc; j++){
        Matrix2 accum1;
        Matrix2 accum2;
        accum1 = scale(partial_delta_u_partial_u(h, j), temp1);
        accum2 = add(accum, accum1);
        for (int i = 0; i < accum2.rows*accum2.cols; i++) accum.data[i] += accum2.data[i];
        release(accum1);
        release(accum2);
      }
    }
    
    release(temp1);
    temp2 = repmat(sub_sum, controller.Nc, 0);
    release(sub_sum);
   
    jacobian = add(accum,temp2);
    release(temp2);
    release(accum);

    for (int h = 0; h < controller.Nc; h++)
    {
      for (int j = 0; j < controller.m; j++)
      {
        for (int i = 0; i < controller.m; i++)
        {
          jacobian.data[i*jacobian.cols + j] += -controller.s / pow(u[j*controller.m + i] + controller.r / 2.0 - controller.b, 2) +  controller.s / pow(controller.r / 2.0 + controller.b - u[j*controller.m + i], 2.0);
        }
      }
    }
    
    return jacobian;
}

Matrix2 get_hessian(Matrix2 del_y, Matrix2 Q, Matrix2 Lambda, Matrix2 ynu, Matrix2 dynu_du, Matrix2 del_u_matrix, float * u, float * del_u){
    Matrix2 hessian;
    Matrix2 temp3;
    Matrix2 temp4;
    Matrix2 temp1;
    Matrix2 temp;
    Matrix2 temp2;
    Matrix2 hessian1;
    Matrix2 trn;
    Matrix2 trn1;
    Matrix2 trn2;
    set(hessian, controller.Nc, controller.Nc);
    
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
    trn = scale(2., temp2);
    release(temp2);
    temp1 = transpose(trn);
    release(trn);
    temp3 = sum_axis(temp, 0);
    release(temp);
    trn1 = sum_axis(temp3, 1);
    release(temp3);
    temp4 = sum_axis(temp1, 0);
    release(temp1);
    trn2 = sum_axis(temp4, 1);
    release(temp4);
    
    for (int i = 0 ; i < controller.Nc*controller.Nc; i++) hessian.data[i] = trn1.data[0] - trn2.data[0]; //  subtract(temp3, temp4);
    release(trn1);
    release(trn2);
 
    set(hessian1, controller.Nc, controller.Nc);
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

    set(second_y, 1, controller.m);
    set(second_y1, controller.m, 1);
    
    for (int h = 0; h < controller.Nc; h++)
    {
      for (int mm = 0; mm < controller.Nc; mm++)
      {
        hessian1.data[h*controller.Nc+mm] = sum2.data[0];
       
        for (int j = 0; j < controller.m ; j++)
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
        hessian1.data[h*controller.Nc + mm] += multtt.data[0];
        release(multtt);
        
         for (int jj = 0; jj < controller.m; jj++)
         {
            for (int i = 0; i < controller.m; i++)
            {
              hessian1.data[h*hessian1.cols + mm] += 2.0* controller.s / pow((u[jj*controller.m + i] + controller.r / 2. - controller.b), 3.0) + 2.0 * controller.s / pow(controller.r/2. + controller.b - u[jj*controller.m + i], 3.0);
            }
        }
      }
    }
  
    release(sum2);
    release(second_y);
    release(second_y1);
    release(mult1);
    release(hessian);
    return hessian1;
}

void solve(Matrix2 jacobian, Matrix2 hessian, Matrix2 &del_u_matrix){
    Matrix2 inv;
    inv = inverse(hessian);
    Matrix2 minusj;
    minusj = scale(-1., jacobian);
    del_u_matrix = multiply(inv, minusj); 
    release(inv);
    release(minusj);
}
