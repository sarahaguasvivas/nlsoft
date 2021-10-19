/* File: helpers.i:

    This is the file that gives us enough to wrap the code. 
    Here we introduce headers that import the functions or header
    files. Since activations.h also imports all of the function 
    prototypes from activation.cpp, you only need to import the 
    header.
*/
%{
    #define SWIG_FILE_WITH_INIT
    #include "activations.h"
    #include "parameters.h"
    #include "helpers.hpp"
    #include "dense.hpp"
    #include "matrix.hpp"
%}
    %include "activations.h"
    %include "parameters.h"
    %include "helpers.hpp"
    %include "dense.hpp"
    %include "matrix.hpp"
    %include "activations.cpp"
    %include "helpers.cpp"
    %include "dense.cpp"
    %include "matrix.cpp"

%include "stdint.i"
%include "carrays.i"

%module helpers

%array_class(float, input);

%{
    #define PI 3.1415926535897932384626433832795
 
    extern void setup_nn_utils();
    extern void nn_gradients(Matrix2 *, Matrix2 *, int, int, int, int, float *, float);
    extern void roll_window(int, int, int, float *);
    extern void normalize_array(float*, float*, int, float);
    extern Matrix2 nn_prediction(int, int, int, int, int, int, int, float*, float*);
    extern void build_input_vector(float *, float * u, float * signal_, float * posish, int ndm, int ddn, int   m, int n, int);
    extern float partial_delta_u_partial_u(int, int);
    extern float kronecker_delta(int, int);
    extern Matrix2 get_jacobian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *, struct  Controller);
    extern Matrix2 get_hessian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *, struct   Controller);
    extern void solve(Matrix2, Matrix2, Matrix2 &);
    extern void clip_action(Matrix2 &, Controller*);
 
     extern void set(struct Matrix2 &, int, int);
     extern void set_to_zero(struct Matrix2 &);
     extern void set_to_ones(struct Matrix2 &);
     extern struct Matrix2 sum_axis(struct Matrix2, int);
     extern struct Matrix2 transpose(struct Matrix2);
     extern struct Matrix2 scale(float, struct Matrix2);
     extern struct Matrix2 add (struct Matrix2, struct Matrix2);
     extern struct Matrix2 subtract (struct Matrix2, struct Matrix2);
     extern struct Matrix2 multiply(struct Matrix2, struct Matrix2);
     extern struct Matrix2 hadamard(struct Matrix2, struct Matrix2);
     extern struct Matrix2 inverse (struct Matrix2);
     extern void release(struct Matrix2 & a);
     extern void lubksb(struct Matrix2, int *, float *);
     extern void ludcmp(struct Matrix2, int *, float *);
     extern struct Matrix2 repmat(struct Matrix2, int, int);
     extern void equal(struct Matrix2 &, struct Matrix2);
     extern struct Matrix2 nr_optimizer(struct Matrix2, struct Matrix2, struct Matrix2);
     extern struct Matrix2 solve_matrix_eqn(struct Matrix2, struct Matrix2);
     extern void cleanup_helper (float **);

    extern float * activate(float* input, int output_shape, char type);

    extern float * sigmoid(float * input, int m);

    extern float * exp_activation(float * input, int m);

    extern float * softplus(float * input, int m);

    extern float * softsign(float * input, int m);

    extern float * hard_sigmoid(float * input, int m);

    extern float  exponential(float input);

    extern float * relu(float *input, int m);

    extern float * hyper_tan(float * input, int m);

    extern float * softmax(float * input, int m);
%}
    #define PI 3.1415926535897932384626433832795
 
    extern void setup_nn_utils();
    extern void nn_gradients(Matrix2 *, Matrix2 *, int, int, int, int, float *, float);
    extern void roll_window(int, int, int, float *);
    extern void normalize_array(float*, float*, int, float);
    extern Matrix2 nn_prediction(int, int, int, int, int, int, int, float*, float*);
    extern void build_input_vector(float *, float * u, float * signal_, float * posish, int ndm, int ddn, int   m, int n, int);
    extern float partial_delta_u_partial_u(int, int);
    extern float kronecker_delta(int, int);
    extern Matrix2 get_jacobian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *, struct  Controller);
    extern Matrix2 get_hessian(Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, Matrix2, float *, float *, struct   Controller);
    extern void solve(Matrix2, Matrix2, Matrix2 &);
    extern void clip_action(Matrix2 &, Controller*);

 
     extern void set(struct Matrix2 &, int, int);
     extern void set_to_zero(struct Matrix2 &);
     extern void set_to_ones(struct Matrix2 &);
     extern struct Matrix2 sum_axis(struct Matrix2, int);
     extern struct Matrix2 transpose(struct Matrix2);
     extern struct Matrix2 scale(float, struct Matrix2);
     extern struct Matrix2 add (struct Matrix2, struct Matrix2);
     extern struct Matrix2 subtract (struct Matrix2, struct Matrix2);
     extern struct Matrix2 multiply(struct Matrix2, struct Matrix2);
     extern struct Matrix2 hadamard(struct Matrix2, struct Matrix2);
     extern struct Matrix2 inverse (struct Matrix2);
     extern void release(struct Matrix2 & a);
     extern void lubksb(struct Matrix2, int *, float *);
     extern void ludcmp(struct Matrix2, int *, float *);
     extern struct Matrix2 repmat(struct Matrix2, int, int);
     extern void equal(struct Matrix2 &, struct Matrix2);
     extern struct Matrix2 nr_optimizer(struct Matrix2, struct Matrix2, struct Matrix2);
     extern struct Matrix2 solve_matrix_eqn(struct Matrix2, struct Matrix2);
     extern void cleanup_helper (float **);

extern float * activate(float* input, int output_shape, char type);

extern float * sigmoid(float * input, int m);

extern float * exp_activation(float * input, int m);

extern float * softplus(float * input, int m);

extern float * softsign(float * input, int m);

extern float * hard_sigmoid(float * input, int m);

extern float  exponential(float input);

extern float * relu(float *input, int m);

extern float * hyper_tan(float * input, int m);

extern float * softmax(float * input, int m);
