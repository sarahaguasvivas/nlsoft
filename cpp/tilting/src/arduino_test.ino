#include "helpers.hpp"
#include "signals.hpp"
#include "matrix.hpp"
#include "motors.hpp"
#include "swirl_target.hpp"

#define NUM_SIGNAL 18
#define NN_INPUT_LENGTH   36

void print_matrix(Matrix2);
void print_array(float *, int);

unsigned long timestamp;
float current_position[3] = { 0.0 };
unsigned long elapsed;

struct Controller controller; // controller struct instantiation

MCUCore core;
CoreChannel core_chnl(0);

long int count = 0;
float h_tm1[GRU_OUTPUT] = {0.0};

void setup() {
  setup_signal_collector();
  setup_nn_utils();
  setup_motors();
  timestamp = 0;
  Serial.begin(115200);
}

void loop() {
    elapsed = millis();
    float * signal = (float*)malloc(NUM_SIGNAL*sizeof(float));
    float * nn_input = (float*)malloc((controller.nn_input_size)*sizeof(float));
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
    for (int i = 0; i < controller.m; i++) Lambda.data[i*controller.m + i] = controller
                                                                            .lambda_matrix[i];
    set(target, controller.N, controller.n);
    if (timestamp == 0) {
      set_to_zero(del_u_matrix);
    } else{
      for (int i = 0; i < controller.Nc*controller.m; i++){
          del_u_matrix.data[i] = controller.del_u[i];
      }
    }
    collect_signal(&signal[0], &controller.signal_calibration[0], 
                                  NUM_SIGNAL);
    for (int i = 0; i < NN_INPUT_LENGTH; i++) {
      nn_input[i] = controller.past_nn_input[i];
    }
    if (timestamp > 0){
      print_array(nn_input, NN_INPUT_LENGTH);
      nn_input = build_input_vector(&nn_input[0], &controller.u[0], 
                         &signal[0], &current_position[0], 
                         controller.nd * controller.m, 
                         controller.dd * controller.n, 
                         controller.m, controller.n, NUM_SIGNAL);
    }
    print_array(nn_input, NN_INPUT_LENGTH);
    Serial.println(); 
    for (int i = 0 ; i < NN_INPUT_LENGTH; i++) {
         controller.past_nn_input[i] = nn_input[i];
    }
    normalize_array(&controller.u[0], &controller.normalized_u[0], 
                                        controller.m*controller.Nc, 1.);
    float* h_tm = (float*)malloc(GRU_OUTPUT * sizeof(float));
    for (int i = 0; i < GRU_OUTPUT; i++){
      h_tm[i] = h_tm1[i];
    }   
    prediction = nn_prediction(controller.N, controller.Nc, controller.n, controller.m, 
                               NN_INPUT_LENGTH, controller.nd, controller.dd, &nn_input[0], 
                                controller.normalized_u, &h_tm[0]);
    print_matrix(prediction);
    Serial.println();
    for (int i = 0; i < GRU_OUTPUT; i++){
      h_tm1[i] = h_tm[i];
    }
    free(h_tm);
    for (int i = 0; i < controller.n; ++i) {
        current_position[i] = prediction.data[(controller.N - 1) * controller.n + i];
    }
    set(ynu, controller.n, controller.m);
    set(dynu_du, controller.n, controller.m);
    nn_gradients(&ynu, &dynu_du, controller.n, controller.m, 
                              controller.nd, controller.nn_input_size, 
                              nn_input, controller.epsilon);
    spin_swirl_target(timestamp, 0, controller.N, 
                              controller.n, &target, controller.neutral_point);
    del_y = subtract(target, prediction);
    for (int i = 0; i < controller.N*controller.n; i++){
       controller.y[i] = prediction.data[i];
    }
    release(prediction);
    release(target);
    //// jacobian ////////////////////////////////////////////////////////////////////
    Matrix2 jacobian;
    jacobian = get_jacobian(del_y, Q, Lambda, ynu, 
                              dynu_du, del_u_matrix, &controller.u[0], 
                              &controller.del_u[0], controller);
    //// hessian /////////////////////////////////////////////////////////////////////
    Matrix2 hessian;
    hessian = get_hessian(del_y, Q, Lambda, ynu, dynu_du, 
                            del_u_matrix, &controller.u[0], 
                              &controller.del_u[0], controller);
    ////////////////////////////////////////////////////////////////////////////////
    release(del_y);
    release(ynu);
    release(dynu_du);
    Matrix2 u_matrix; 
    solve(jacobian, hessian, del_u_matrix);
    set(u_matrix, controller.Nc, controller.m);
    for (int i = 0; i < controller.Nc*controller.m; i++) { 
      u_matrix.data[i] = controller.prev_u[i] - del_u_matrix.data[i];
    }
    clip_action(u_matrix, &controller);
    for (int i = 0; i < controller.Nc*controller.m; i++) {
      controller.prev_u[i] = controller.u[i];
      controller.u[i] = u_matrix.data[i];
      controller.del_u[i] = del_u_matrix.data[i];
    }
    print_matrix(u_matrix);
    step_motor(&u_matrix.data[0], controller.m);
    release(u_matrix);
    release(hessian);
    release(jacobian);
    release(Q);
    release(Lambda);
    release(del_u_matrix);
    free(signal);
    free(nn_input);
    timestamp++;
    Serial.println(millis() - elapsed);
}

void print_array(float * arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++) {
      Serial.print(arr[i], 10);
      Serial.print(",");
    }
    Serial.println();
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
}
