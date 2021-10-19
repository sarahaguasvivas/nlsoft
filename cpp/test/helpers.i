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

