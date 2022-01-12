#include "helpers.hpp"
#include "signals.hpp"
#include "matrix.hpp"
#include "motors.hpp"
#include "figure_eight_target.hpp"

#define NUM_SIGNAL 6

unsigned long timestamp;
float current_position[3] = { 0.0,
                             0.0, 
                             0.0};
//unsigned long elapsed;

Controller controller; // controller struct instantiation

MCUCore core;
CoreChannel core_chnl(0);

//AnalogInChannel chnl_13(13, A4); // was used for laser displacement sensor

void setup() {
  setup_signal_collector();
  setup_nn_utils();
  timestamp = 0;
  Serial.begin(2000000);
}

void loop() {


    //elapsed = millis();
    //float * signal_ = (float*)malloc(NUM_SIGNAL*sizeof(float));
    //Matrix2 Q;
    //Matrix2 Lambda;
    //Matrix2 del_u_matrix;
    //Matrix2 del_y;
    //Matrix2 target;
    //Matrix2 prediction;
    //Matrix2 ynu;
    //Matrix2 dynu_du;
    
    //set(Q, controller.n, controller.n);
    //set(Lambda, controller.m, controller.m);
    //set(del_u_matrix, controller.Nc, controller.m);
    //set_to_zero(Q);
    //set_to_zero(Lambda);
    
    //for (int i = 0; i < controller.n; i++) Q.data[i*controller.n+i] = controller.q_matrix[i];
    //for (int i = 0; i < controller.m; i++) Lambda.data[i*controller.m + i] = controller
    //                                                                        .lambda_matrix[i];
    
    //set(target, controller.N, controller.n);
    //float * nn_input = (float*)malloc((controller.nn_input_size)*sizeof(float));
    
    //if (timestamp == 0) {
    //  set_to_zero(del_u_matrix);
    //} else{
    //  for (int i = 0; i < controller.Nc*controller.m; i++){
    //      del_u_matrix.data[i] = controller.del_u[i];
    //  }
    //}
    
    //for (int i = 0; i < controller.nn_input_size; i++) {
    //  nn_input[i] = controller.past_nn_input[i];
    //}
    //delay(1); // 3
    //collect_signal(&signal_[0], &controller.signal_calibration[0], NUM_SIGNAL);
    
    //if (timestamp > 0){
    //  build_input_vector(nn_input, controller.normalized_u, signal_, current_position, 
    //                    controller.nd*controller.m, controller.dd*controller.n, 
    //                    controller.m, controller.n, NUM_SIGNAL);
    //} 

    //for (int i = 0 ; i < controller.nn_input_size ; i++) {
    //  controller.past_nn_input[i] = nn_input[i];
    //}

    //normalize_array(&controller.u[0], &controller.normalized_u[0], 
    //                                    controller.m*controller.Nc, PI);
   
    //prediction = nn_prediction(controller.N, controller.Nc, controller.n, controller.m, 
    //                           NUM_SIGNAL + controller.nd*controller.m + controller.dd*controller.n, 
    //                           controller.nd, controller.dd, nn_input, controller.normalized_u);
    //for (int i = 0; i < controller.n; i++) {
    //    current_position[i] = prediction.data[(controller.N - 1) + i * controller.n];
    //}
    //set(ynu, controller.n, controller.m);
    //set(dynu_du, controller.n, controller.m);
    
    //nn_gradients(&ynu, &dynu_du, controller.n, controller.m, 
    //                          controller.nd, controller.nn_input_size, 
    //                          nn_input, controller.epsilon);
    
    //spin_figure_eight_target(timestamp, 0, controller.N, 
    //                          controller.n, &target, controller.neutral_point);
    //del_y = subtract(target, prediction);
    
    //for (int i = 0; i < controller.N*controller.n; i++){
    //   controller.y[i] = prediction.data[i];
    //}

    //release(prediction);
    //release(target);
    
    //// jacobian ////////////////////////////////////////////////////////////////////
    //Matrix2 jacobian;
    //jacobian = get_jacobian(del_y, Q, Lambda, ynu, 
    //                          dynu_du, del_u_matrix, controller.u, 
    //                          controller.del_u, controller);
    
    //// hessian /////////////////////////////////////////////////////////////////////
    //Matrix2 hessian;
    //hessian = get_hessian(del_y, Q, Lambda, ynu, dynu_du, 
    //                        del_u_matrix, controller.u, 
    //                          controller.del_u, controller);
    //////////////////////////////////////////////////////////////////////////////////
    //release(del_y);
    
    //Matrix2 u_matrix; 

    //solve(jacobian, hessian, del_u_matrix);
    //set(u_matrix, del_u_matrix.rows, del_u_matrix.cols);
    //for (int i = 0; i < controller.Nc*controller.m; i++) { 
    //  u_matrix.data[i] = controller.prev_u[i] - del_u_matrix.data[i];
    //  if (isnan(u_matrix.data[i])){
    //    u_matrix.data[i] = -1.308;
    //  }
    //}
    //clip_action(u_matrix, &controller);
    
    //for (int i = 0; i < controller.Nc*controller.m; i++) { 
    //  controller.prev_u[i] = controller.u[i];
    //  controller.u[i] = u_matrix.data[i];
    //  controller.del_u[i] = del_u_matrix.data[i];
    //}
    
    //step_motor(&u_matrix.data[0], controller.m);

    //release(hessian);
    //release(jacobian);
    //release(Q);
    //release(Lambda);
    //release(u_matrix);
    //release(del_u_matrix);
    
    //free(nn_input);
    //free(signal_);
    //timestamp++;
    //Serial.println(millis() - elapsed);
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
