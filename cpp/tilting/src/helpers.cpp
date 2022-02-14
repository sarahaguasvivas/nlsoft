#include "helpers.hpp"

float max (float a, float b) {
  return (a<b)?b:a;     // or: return comp(a,b)?b:a; for version (2)
}
float min (float a, float b) {
  return (a>b)?b:a;     // or: return comp(a,b)?b:a; for version (2)
}

void setup_nn_utils()
{
  buildLayers();  
}

float deg2rad(float deg)
{
    return deg*PI/180.;
}

void normalize_array(float* original_array, float* new_array, int array_size, float divisor){
  for (int i= 0; i< array_size; i++){
    new_array[i] = original_array[i] / divisor; 
  }
}

void clip_action(
            Matrix2 &u_matrix, 
            Controller* controller
){
  for (int i = 0; i < controller->Nc*controller->m; i++){
    u_matrix.data[i] = min(u_matrix.data[i], controller->min_max_input_saturation[1]); 
    u_matrix.data[i] = max(u_matrix.data[i], controller->min_max_input_saturation[0]); 
  }
}

void roll_window(int start, int end, int buffer_size, float * array)
{
  for (int i = end; i >= start + buffer_size; i--) 
  {
      array[i] = array[i - buffer_size];
  }
}

float* build_input_vector(
                  float * vector, 
                  float * u,  
                  float * sensors, 
                  float * posish, 
                  int ndm, 
                  int ddn, 
                  int m, 
                  int n, 
                  int num_sig
){
  int input_size = ndm + ddn + num_sig;
  roll_window(ndm, ddn + ndm - 1, n, vector);
  for (int i = 0; i < n; i++){
     vector[i + ndm] = posish[i];
  }
  roll_window(0, ndm - 1, m, vector);
  for (int i = 0; i < m; i++){
      vector[i] = u[i];
  }   
  for (int i = ndm + ddn; i < input_size; i++){ 
    vector[i] = sensors[i - ndm - ddn];
  }
  return vector; 
}

Matrix2 nn_prediction(
              int N, 
              int Nc, 
              int n, 
              int m, 
              int input_size, 
              int nd, 
              int dd, 
              float * prev_input, 
              float * u,
              float * h_tm1
){
    float * previous_input = (float*)malloc(input_size*sizeof(float));
    for (int i = 0; i < input_size; i++){
      previous_input[i] = prev_input[i];
    }
    Matrix2 y_output;
    set(y_output, N, n);
    set_to_zero(y_output);
    float * signals = (float*)malloc(NUM_SIGNAL*sizeof(float));
    for (int i = 0; i < NUM_SIGNAL; i++) {
      signals[i] = previous_input[nd*m+dd*n + i];
    }
    const int mnd = m*nd;
    float motor_input[mnd];
    for (int i = 0; i < N; i++)
    {
        float * input_next = (float*)malloc(input_size*sizeof(float));        
        for (int j = 0; j < input_size; j++) {
          input_next[j] = previous_input[j];
        }
        
        float * output_next;
        output_next = fwdNN(input_next, h_tm1);
        
        int input_index = (i < Nc) ? i : (Nc-1);
        
        for (int k = 0; k < m; k++)
        {
           motor_input[k] = u[input_index*m + k];
        } 
        previous_input = build_input_vector(previous_input, 
                                      motor_input, signals, 
                                      output_next, nd*m, dd*n, 
                                      m, n, NUM_SIGNAL);
        for (int j = 0; j < n; j++)
        {
            y_output.data[i * n + j] = output_next[j];
        }
        free(output_next);
    }
    free(signals);
    free(previous_input);
    return y_output;
}

void nn_gradients(Matrix2 * first_derivative, Matrix2 * second_derivative, 
                  const int n, int m, int nd, int input_size, 
                  float * input, float epsilon)
{
    set_to_zero(*first_derivative);
    set_to_zero(*second_derivative);
    float * h_tm1 = (float*)malloc(GRU_OUTPUT*sizeof(float)); 
    for (int i = 0; i < GRU_OUTPUT; i++){
        h_tm1[i] = 0.0;
    }
    for (int nn = 0; nn < nd*m; nn++)
    {
      float * output;
      float * output_minus_h;
      float * output_plus_h;  

      float * input_center = (float*)malloc(input_size*sizeof(float));
      float * input_plus_h = (float*)malloc(input_size*sizeof(float));
      float * input_minus_h = (float*)malloc(input_size*sizeof(float));

      for (int i = 0; i < input_size; i++)
      {
        input_center[i] =  input[i];
        input_plus_h[i] =  input[i];
        input_minus_h[i] = input[i];            
      }
      input_plus_h[nn] += epsilon;
      input_minus_h[nn] -= epsilon;

      output_plus_h = fwdNN(input_plus_h, h_tm1);
      output_minus_h = fwdNN(input_minus_h, h_tm1);
      output = fwdNN(input_center, h_tm1);
      int j = nn % m;
      for (int i = 0 ; i < n; i++){
        first_derivative->data[i*first_derivative->cols + j] += 
                                (output_plus_h[i] - output_minus_h[i]) / (2. * epsilon);
        second_derivative->data[i*second_derivative->cols + j] += 
                              (output_plus_h[i] - 2.*output[i] + output_minus_h[i]) / (epsilon * epsilon);    
      }
      free(output);
      free(output_minus_h);
      free(output_plus_h);  
    }
    free(h_tm1);
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

Matrix2 get_jacobian(Matrix2 del_y, Matrix2 Q, Matrix2 Lambda, Matrix2 ynu, 
                        Matrix2 dynu_du, Matrix2 del_u_matrix, float * u, 
                          float * del_u, struct Controller controller){
    Matrix2 jacobian;
    Matrix2 sub_sum;
    Matrix2 temp;
    Matrix2 temp1;
    Matrix2 temp2;
    Matrix2 accum;
    set(accum, controller.Nc, controller.m);
    set_to_zero(accum);
    set_to_zero(jacobian); 
    sub_sum = multiply(del_y, Q);
    temp = multiply(sub_sum, ynu);
    release(sub_sum);
    sub_sum = sum_axis(temp, 0);
    release(temp);
    
    temp = multiply(del_u_matrix, Lambda);
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
          jacobian.data[h*jacobian.cols + j] += -controller.s / pow(u[j*controller.m + i] + 
                                                  controller.r / 2.0 - controller.b, 2) +  controller.s / 
                                                  pow(controller.r / 2.0 + controller.b - u[j*controller.m + i], 2.0);
        }
      }
    }
     
    return jacobian;
}

Matrix2 get_hessian(Matrix2 del_y, Matrix2 Q, Matrix2 Lambda, Matrix2 ynu, 
                          Matrix2 dynu_du, Matrix2 del_u_matrix, float * u, float * del_u, 
                            struct Controller controller){
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
    set_to_zero(hessian);
     
    temp = hadamard(ynu, ynu);
    temp1 = multiply(Q, temp);
    release(temp);
    temp = scale(2., temp1);
    release(temp1);
    temp1 = multiply(Q, dynu_du);
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
     
    for (int i = 0 ; i < controller.Nc*controller.Nc; i++) {
      hessian.data[i] = trn1.data[0] - trn1.data[0];  
    }
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
    sum1 = sum_axis(hessian1, 0);
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
        hessian1.data[h*controller.Nc + mm] += multtt.data[0] + controller.machine_zero;
        release(multtt);
         
        for (int jj = 0; jj < controller.m; jj++)
        {
           for (int i = 0; i < controller.m; i++)
           {
            hessian1.data[h*hessian1.cols + mm] += 2.0* controller.s / 
                    pow((u[jj*controller.m + i] + controller.machine_zero + 
                    controller.r / 2. - controller.b), 3.0) + 
                     2.0 * controller.s / pow(controller.r/2. + controller.b - 
                     (u[jj*controller.m + i] + 
                    controller.machine_zero), 3.0) + controller.machine_zero;
           }
        }
       release(mult1);
      }
    }
    release(sum2);
    release(second_y);
    release(second_y1);
    release(hessian);
    return hessian1;
}

void solve(Matrix2& jacobian, Matrix2& hessian, Matrix2 &del_u_matrix){
    Matrix2 inv;
    inv = inverse(hessian);
    Matrix2 minusj;
    minusj = scale(-1., jacobian);
    release(del_u_matrix);
    del_u_matrix = multiply(inv, minusj); 
    release(inv);
    release(minusj);
}
