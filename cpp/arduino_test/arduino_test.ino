#include "neural_network_utilities.hpp"
#include "matrix.hpp"
#include "motors.hpp"
#include "signals.hpp"
#include "figure_eight_target.hpp"

#define NUM_SIGNAL  11
#define PI 3.1415926535897932384626433832795

unsigned long timestamp;
float posish[3] = {-0.06709795916817293, -0.047865542156502586, -0.016102764150255758};
unsigned long elapsed;

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
    jacobian = get_jacobian(del_y, Q, Lambda, ynu, dynu_du, del_u_matrix, controller.u, controller.del_u, controller);
    
    // hessian /////////////////////////////////////////////////////////////////////
    Matrix2 hessian;
    hessian = get_hessian(del_y, Q, Lambda, ynu, dynu_du, del_u_matrix, controller.u, controller.del_u, controller);
    
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
