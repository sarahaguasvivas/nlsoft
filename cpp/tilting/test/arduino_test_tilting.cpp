#include <iostream>
#include "helpers.hpp"
//#include "signals.hpp"
#include "matrix.hpp"
//#include "motors.hpp"
#include "swirl_target.hpp"

#define NUM_SIGNAL 18
#define NN_INPUT_LENGTH   36

void print_matrix(Matrix2);
void print_array(float *, int);

unsigned long timestamp;
float current_position[3] = { 0.0, 0.0, 0.0 };
unsigned long elapsed;

struct Controller controller; // controller struct instantiation

//MCUCore core;
//CoreChannel core_chnl(0);

long int count = 0;
//int actuators_turn = 0;
//float h_tm1[GRU_OUTPUT] = {0.0};

void setup() {
  //setup_signal_collector();
  setup_nn_utils();
 // setup_motors();
  timestamp = 0;
  //Serial.begin(115200);
}

int main() {
    setup();
    for (int i = 0; i < 20; i++){
      float * signal = (float*)malloc(
                              NUM_SIGNAL*sizeof(float));
      float * nn_input = (float*)malloc(
                              (controller.nn_input_size)*sizeof(float));
      for (int i = 0; i < controller.nn_input_size; i++){
        nn_input[i] = 0.0;
      }
      struct Matrix2 Q, Lambda, del_u_matrix, del_y, target, prediction, 
                  ynu, dynu_du;
      set(Q, controller.n, controller.n);
      set(Lambda, controller.m, controller.m);
      set(del_u_matrix, controller.Nc, controller.m);
      
      for (int i = 0; i < controller.n; i++) Q.data[i*controller.n+i] = controller.q_matrix[i];
      for (int i = 0; i < controller.m; i++) Lambda.data[i*controller.m + i] = controller
                                                                              .lambda_matrix[i];
      set(target, controller.N, controller.n);
      for (int i = 0; i < controller.Nc*controller.m; i++){
          del_u_matrix.data[i] = controller.del_u[i];
      }
      //collect_signal(&signal[0], controller.signal_calibration, 
      //                             NUM_SIGNAL);
      for (int i = 0; i < NUM_SIGNAL; i++){
        signal[i] = 0.0;
      }
      //print_array(signal, NUM_CHANNELS);
      nn_input = build_input_vector(&nn_input[0], &controller.u[0], 
                          &signal[0], &current_position[0], 
                          controller.nd * controller.m, 
                          controller.dd * controller.n, 
                          controller.m, controller.n, NUM_SIGNAL);

      if (timestamp == 0){
        for (int i = 0; i < controller.m; i++){
          nn_input[i + controller.m] = controller.u[i];
        }
      } 
      //float* h_tm = (float*)malloc(GRU_OUTPUT * sizeof(float));
      //for (int i = 0; i < GRU_OUTPUT; i++){
      //  h_tm[i] = h_tm1[i];
      //} 
      //print_array(nn_input, NN_INPUT_LENGTH);  
      prediction = nn_prediction(controller.N, controller.Nc, controller.n, controller.m, 
                                NN_INPUT_LENGTH, controller.nd, controller.dd, &nn_input[0], 
                                &controller.u[0], &controller.neutral_point[0]);
      //print_matrix(prediction); 
      //print_with_scale(prediction, 1000.); 
      //for (int i = 0; i < GRU_OUTPUT; i++){
      //  h_tm1[i] = h_tm[i];
      //}
      
      set(ynu, controller.n, controller.m);
      set(dynu_du, controller.n, controller.m);
      nn_gradients(&ynu, &dynu_du, controller.n, controller.m, 
                                controller.nd, controller.nn_input_size, 
                                nn_input, controller.epsilon);
      //free(h_tm);
      //Serial.println("ynu");
      //print_matrix(ynu);
      //Serial.println("dynu_du");
      //print_matrix(dynu_du);
      spin_swirl_target(timestamp, 0, controller.N, 
                                controller.n, &target, controller.neutral_point, 
                                          controller.wavelength);
      for (int i = 0; i < controller.n; i++) {
          current_position[i] = prediction.data[(controller.N - 1) * controller.n + i];
      }
      //print_with_scale(prediction, 1000.);
      //print_two_arrays(prediction.data, controller.n, target.data, controller.n, 1000.); 
      del_y = subtract(target, prediction);
      //print_with_scale(del_y, 1000.);
      release(prediction);
      release(target);
      //// jacobian ////////////////////////////////////////////////////////////////////
      Matrix2 jacobian;
      jacobian = get_jacobian(del_y, Q, Lambda, ynu, 
                                dynu_du, del_u_matrix, controller.prev_u, 
                                controller.del_u, controller);
      //Serial.println("jacobian");
      //print_matrix(jacobian);
      //// hessian /////////////////////////////////////////////////////////////////////
      // TODO(sarahaguasvivas): Hessian too large
      Matrix2 hessian;
      hessian = get_hessian(del_y, Q, Lambda, ynu, dynu_du, 
                              del_u_matrix, controller.prev_u, 
                                controller.del_u, controller);
      //Serial.println("hessian");
      //print_matrix(hessian);
      ////////////////////////////////////////////////////////////////////////////////
      release(del_y);
      release(ynu);
      release(dynu_du);
      Matrix2 u_matrix; 
      solve(jacobian, hessian, &del_u_matrix);
      set(u_matrix, controller.Nc, controller.m);
      for (int i = 0; i < controller.Nc*controller.m; i++) { 
        u_matrix.data[i] = controller.prev_u[i] + del_u_matrix.data[i];
        if (isnan(u_matrix.data[i])){
          u_matrix.data[i] = controller.min_max_input_saturation[0];
        }
      }
      clip_action(u_matrix, &controller);
      for (int i = 0; i < controller.m; i++) {
        controller.prev_u[i] = u_matrix.data[i];
        controller.u[i] = u_matrix.data[i];
        controller.del_u[i] = del_u_matrix.data[i];
      }
      //print_matrix(u_matrix);
      //step_motor(&u_matrix.data[0], controller.m, 
      //            controller.input_calibration, controller.input_offset);
      release(u_matrix);
      release(hessian);
      release(jacobian);
      release(Q);
      release(Lambda);
      release(del_u_matrix);
      free(signal);
      free(nn_input);
      timestamp++;
   } 
    ////Serial.println(millis() - elapsed);
    return 0;
}

void print_array(float * arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++) {
        std::cout << arr[i] << " ";
    }
    std::cout << std::endl;
}

void print_matrix(Matrix2 matrix)
{
  for (int i=0; i< matrix.rows; i++){
    for (int j=0; j< matrix.cols; j++){
        std::cout << matrix.data[i*matrix.cols + j] << " ";
    }
    //Serial.println();
    std::cout << std::endl;
  }
}
