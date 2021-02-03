
/********************
    activations.cpp

    Code generated using nn4mc.

    This file implements all activation functions.

*/

#include "activations.h"
#include <math.h>
#include <stdlib.h>

#define max(a, b) (((a)>(b) ? (a) : (b)))
#define min(a, b) (((a)<(b) ? (a) : (b)))

float sigmoid(float input)
{
  if (input >= 0.0){
	input = 1./ (1. + exp(-1.* input));
  } else{	
  	input = exp(input)/(exp(input) + 1.);
  }
  return input;
}

float softplus(float input)
{
  input = log(exp(input) + 1);

  return input;
}

float softsign(float input)
{
  input = input / (abs(input) + 1);

  return input;
}

float hard_sigmoid(float input)
{

  input = 0.2*input + 0.5;
  if (input <= 0.0){
	input = 0.0;
  }else{
	if (input >= 1.){
	    input = 1.0;
	}
  } 
  return input;
}

float exponential(float input)
{
  input = (float)expf((float)input);

  return input;
}

float elu(float input, float alpha)
{
  if (input > 0){
    input = 0.0;
  } else{
	input = alpha * (exp(input) - 1.);
  }
  return input;
}

float selu(float input)
{
  int scale = 1.0507009873554804934193349852946;
  int alpha = 1.6732632423543772848170429916717; 
  if (input > 0.0) {
      input *= scale; 
  } else{
 	input = (exp(input) - 1.) * scale * alpha;
	}
  return input;
}

float relu(float input)
{
  input = max(input, 0.0);

  return input;
}

float tanh(float input)
{
  return tanh(input);
}

float softmax(float input, int output_shape)
{
  float sum_exp = 0.0;
  for (int i=0; i<output_shape; i++){
      sum_exp+= expf(input);
  }
  for (int i=0; i<output_shape;i++){
      float calc = expf(input) / sum_exp;
      if (isnan(calc)){
          input = 1.0;
      } else input = (float)(expf(input) / sum_exp);
  }

  return input;
}

//NOTE: This is deprecated and need to be changed to be more streamlined
float activate(float input, int output_shape, char type)
{
  if (type == 0x00)
    return softmax(input, output_shape);

   else if (type == 0x02)
    return elu(input, 0.5);
	
   else if (type == 0x03)
    return selu(input); 

  else if (type == 0x04)
    return softplus(input);

  else if (type == 0x05)
    return softsign(input);

  else if (type == 0x06)
    return relu(input);

  else if (type == 0x07)
    return tanh(input);

  else if (type == 0x08)
    return sigmoid(input);

  else if (type == 0x09)
    return hard_sigmoid(input);

  else if (type == 0xA)
    return exponential(input);

  return input;
}
