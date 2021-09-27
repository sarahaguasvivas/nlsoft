#include "figure_eight_target.hpp"

void spin_figure_eight_target(int timestep, int n1, int n2, int dims, Matrix2 * target, float * center)
{
  float wavelength = 400;
  float a = 8./1000.;
  float b = 15./1000.;
  int n = target->cols;
  
  Matrix2 temp;
  //Matrix2 rot_mat;
  set(temp, target->rows, target->cols);
  //set(rot_mat, target->cols, target->cols);
  //set_to_zero(rot_mat);

  //rot_mat.data[2*n + 2] = 1.;
  //rot_mat.data[0] = cos(0.5);
  //rot_mat.data[1] = -sin(0.5);
  //rot_mat.data[1*n + 0] = sin(0.5);
  //rot_mat.data[1*n + 1] = cos(0.5);
  
  for (int i = n1; i < n2; i++)
  {
    temp.data[i*n + 0] = 0.001*sin((timestep+i) / (wavelength))* sin((timestep + i) / (wavelength));
    temp.data[i*n + 1] = a * sin((timestep + i) / wavelength);
    temp.data[i*n + 2] = b * sin((timestep + i) / wavelength) *  cos((timestep + i)/wavelength); 
  }
  
  //Matrix2 temp1;
  //Matrix2 temp2;
  //Matrix2 temp3;

  //temp2 = transpose(temp);
  //temp1 = multiply(rot_mat, temp2);
  //temp3 = transpose(temp1);
    
  for (int i = n1; i < n2; i++)
  {
    temp.data[i*n + 0] += center[0]; 
    temp.data[i*n + 1] += center[1];
    temp.data[i*n + 2] += center[2]; 
  }

  for (int i = 0 ; i < target->rows*n; i++) target->data[i] = temp.data[i];

  //release(temp1);
  //release(temp3);
  //release(temp2);
  //release(rot_mat);
  release(temp);
}
