#include <iostream>
#include "helpers.hpp"
//#include "signals.hpp"
#include "matrix.hpp"
//#include "motors.hpp"
#include "swirl_target.hpp"

#define NUM_SIGNAL 18

void print_matrix(Matrix2);
void print_array(float *, int);


unsigned long timestamp;
float current_position[3] = { 0.0 };
unsigned long elapsed;

Controller controller; // controller struct instantiation

//MCUCore core;
//CoreChannel core_chnl(0);

long int count = 0;
int actuators_turn = 0;

void setup() {
 // setup_signal_collector();
  setup_nn_utils();
 // setup_motors();
  timestamp = 0;
  //Serial.begin(115200);
}

int main() {
    setup();
    //elapsed = millis();
    float * signal = (float*)malloc(NUM_SIGNAL*sizeof(float));
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
    float * nn_input = (float*)malloc((controller.nn_input_size)*sizeof(float));
    
    if (timestamp == 0) {
      set_to_zero(del_u_matrix);
    } else{
      for (int i = 0; i < controller.Nc*controller.m; i++){
          del_u_matrix.data[i] = controller.del_u[i];
      }
    }
    for (int i = 0; i < controller.nn_input_size; i++) {
      nn_input[i] = controller.past_nn_input[i];
    }
    //collect_signal(&signal[0], &controller.signal_calibration[0], NUM_SIGNAL);

    if (timestamp > 0){
      build_input_vector(nn_input, controller.normalized_u, signal, current_position, 
                        controller.nd*controller.m, controller.dd*controller.n, 
                        controller.m, controller.n, NUM_SIGNAL);
    } 

    for (int i = 0 ; i < controller.nn_input_size ; i++) {
      controller.past_nn_input[i] = nn_input[i];
    }
    normalize_array(&controller.u[0], &controller.normalized_u[0], 
                                        controller.m*controller.Nc, PI);
   
    prediction = nn_prediction(controller.N, controller.Nc, controller.n, controller.m, 
                               NUM_SIGNAL + controller.nd*controller.m + controller.dd*controller.n, 
                               controller.nd, controller.dd, nn_input, controller.normalized_u);
    for (int i = 0; i < controller.n; i++) {
        current_position[i] = prediction.data[(controller.N - 1) + i * controller.n];
    }
    set(ynu, controller.n, controller.m);
    set(dynu_du, controller.n, controller.m);
    nn_gradients(&ynu, &dynu_du, controller.n, controller.m, 
                              controller.nd, controller.nn_input_size, 
                              nn_input, controller.epsilon);
    //Serial.println("hhhereerere");
    spin_swirl_target(timestamp, 0, controller.N, 
                              controller.n, &target, controller.neutral_point);
    del_y = subtract(target, prediction);
    //Serial.println("hhhh"); 
    for (int i = 0; i < controller.N*controller.n; i++){
       controller.y[i] = prediction.data[i];
    }
    release(prediction);
    release(target);
    // jacobian ////////////////////////////////////////////////////////////////////
    Matrix2 jacobian;
    jacobian = get_jacobian(del_y, Q, Lambda, ynu, 
                              dynu_du, del_u_matrix, &controller.u[0], 
                              &controller.del_u[0], controller);
    
    // hessian /////////////////////////////////////////////////////////////////////
    Matrix2 hessian;
    hessian = get_hessian(del_y, Q, Lambda, ynu, dynu_du, 
                            del_u_matrix, &controller.u[0], 
                              &controller.del_u[0], controller);
    ////////////////////////////////////////////////////////////////////////////////
    release(del_y);
    Matrix2 u_matrix; 
    solve(jacobian, hessian, del_u_matrix);
    set(u_matrix, controller.Nc, controller.m);
    for (int i = 0; i < controller.Nc*controller.m; i++) { 
      u_matrix.data[i] = controller.prev_u[i] - del_u_matrix.data[i];
      //if (isnan(u_matrix.data[i])){
      //  Serial.println("here");
      //  u_matrix.data[i] = controller.min_max_input_saturation[0];
      //}
    }
    clip_action(u_matrix, &controller);
    for (int i = 0; i < controller.Nc*controller.m; i++) {
      controller.prev_u[i] = controller.u[i];
      controller.u[i] = u_matrix.data[i];
      controller.del_u[i] = del_u_matrix.data[i];
    }

    print_matrix(u_matrix);
    
    //step_motor(&u_matrix.data[0], controller.m);
    release(u_matrix);

    release(hessian);
    release(jacobian);
    
    release(Q);
    release(Lambda);
    release(del_u_matrix);
    
    free(nn_input);
    free(signal);
    timestamp++;
    //Serial.println(millis() - elapsed);
    return 0;
}

void print_array(float * arr, int arr_size)
{
    for (int i = 0; i < arr_size; i++) {
        std::cout << arr[i] << std::endl;
        std::cout << std::endl; 
    }
    std::cout << std::endl;
}

void print_matrix(Matrix2 matrix)
{
  for (int i=0; i< matrix.rows; i++){
    for (int j=0; j< matrix.cols; j++){
        std::cout << matrix.data[i*matrix.cols + j] << std::endl;
        std::cout << std::endl; 
    }
    //Serial.println();
    std::cout << std::endl;
  }
}
