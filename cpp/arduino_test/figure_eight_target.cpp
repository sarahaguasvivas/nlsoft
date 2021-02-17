#include "figure_eight_target.hpp"

void spin_figure_eight_target(int timestep, int n1, int n2, int dims, Matrix2 * target, float * center)
{
  float wavelength = 400;
  float a = 8./1000.;
  float b = 15./1000.;
  
  for (int i=n1; i<n2; i++)
  {
    float x =  0.001*sin((timestep+i) / (wavelength))* sin((timestep+i) / (wavelength));
    float y = a * sin((timestep + i) / wavelength);
    float z = b * sin((timestep + i) / wavelength) *  cos((timestep + i)/wavelength); 

    target->data[i*target->cols + 0] = x;
    target->data[i*target->cols + 1] = y;
    target->data[i*target->cols + 2] = z;
  }
}
