#include "matrix.hpp"

void set(struct Matrix2 & a, int rows, int cols)
{
    a.rows = rows;
    a.cols = cols; 
    if (a.rows > 0 && a.cols > 0) a.data = (float*)malloc(a.rows*a.cols*sizeof(float));
    set_to_zero(a);
}

void cleanup_helper (float ** pointer){
  free(*pointer);
  *pointer = NULL;
}

void release(struct Matrix2 & a)
{
    cleanup_helper(&a.data);
}

struct Matrix2 scale(float scale, struct Matrix2 matrix)
{
    Matrix2 scaled;
    set(scaled, matrix.rows, matrix.cols);
    set_to_zero(scaled);
    for (int i=0; i< matrix.rows; i++)
      for (int j=0; j<matrix.cols;j++)
        scaled.data[i*scaled.cols + j] = scale*matrix.data[i*matrix.cols + j];
    return scaled;
}

struct Matrix2 repmat(struct Matrix2 a, int rep, int axis){
  Matrix2 repmat;

  if (axis == 0){
    set(repmat, a.rows*rep, a.cols);
    for (int r = 0; r < rep; r++){
      for (int i = 0; i < a.rows; i ++){
        for (int j = 0; j < a.cols; j++){
          repmat.data[(i+r*a.rows)*a.cols + j] = a.data[i*a.cols+j];
        }
      }
    }
  } else{
    set(repmat, a.rows, a.cols*rep);
    for (int r = 0; r < rep; r++){
      for (int i = 0; i < a.rows; i ++){
        for (int j = 0; j < a.cols; j++){
          repmat.data[i*(rep*a.cols) + j + r*(a.cols)] = a.data[i*a.cols+j];
        }
      }
    }
  }
  
  return repmat;
}

struct Matrix2 transpose(struct Matrix2 a)
{
    int size = a.rows*a.cols;
    
    Matrix2 transp;
    set(transp, a.cols, a.rows);
    set_to_zero(transp);

    for (int i = 0 ; i< a.rows; i++)
    {
        for (int j = 0; j< a.cols; j++)
        {
            transp.data[j * a.rows + i] = a.data[i * a.cols + j];
        }
    }
    
    return transp;
}

struct Matrix2 add (struct Matrix2 a, struct Matrix2 b)
{
    Matrix2 result;
    set(result, a.rows, a.cols);

    if (a.rows == b.rows && a.cols == b.cols)
    {
        for (int i=0; i< a.rows; i++)
        {
            for (int j=0; j< a.cols; j++)
            {
                int flat_idx = i*a.cols + j;
                result.data[flat_idx] = a.data[flat_idx] + b.data[flat_idx];
            }
        }
    }
    return result;
}

struct Matrix2 sum_axis(struct Matrix2 a, int axis){
  struct Matrix2 sum_axis;
  
  if (axis == 0){
    set(sum_axis, 1, a.cols);
    set_to_zero(sum_axis);
    for (int i = 0 ; i < a.rows; i++){
      for (int j = 0; j < a.cols; j++){
        int index_original = i*a.cols + j;
        sum_axis.data[j] += a.data[index_original];
      }
    }
  } 
  if (axis == 1){
    set(sum_axis, a.rows, 1);
    set_to_zero(sum_axis);
    for (int i = 0; i < a.rows; i++){
      for (int j = 0; j < a.cols; j++){
        int index_original = i*a.cols+j;
        int index_sum = i*a.cols;
        sum_axis.data[index_sum] = a.data[index_original];
      }
    }
  }
  return sum_axis;
}

struct Matrix2 subtract (struct Matrix2 a, struct Matrix2 b)
{
    Matrix2 result;
    set(result, a.rows, a.cols);

    if (a.rows == b.rows && a.cols == b.cols)
    {
        for (int i=0; i< a.rows; i++)
        {
            for (int j=0; j< a.cols; j++)
            {
                int flat_idx = i*a.cols + j;
                result.data[flat_idx] = a.data[flat_idx] - b.data[flat_idx];
            }
        }
    }
    return result;
}

struct Matrix2 hadamard (struct Matrix2 a, struct Matrix2 b)
{
    Matrix2 result;
    set(result, a.rows, a.cols);

    if (a.rows == b.rows && a.cols == b.cols)
    {
        for (int i=0; i< a.rows; i++)
        {
            for (int j=0; j< a.cols; j++)
            {
                int flat_idx = i*a.cols + j;
                result.data[flat_idx] = a.data[flat_idx] * b.data[flat_idx];
            }
        }
    }
    return result;
}

float total_sum(struct Matrix2 a){
    int total_sum = 0;
    for (int i = 0; i < a.cols*a.rows; i++){
        total_sum += a.data[i];
    }
    return total_sum;
}

void set_to_zero(struct Matrix2 & a){
    int size = a.rows*a.cols;
    for (int i = size - 1; i >= 0; i--) a.data[i] = 0.0;
}

void set_to_ones(struct Matrix2 & a){
    int size = a.rows*a.cols;
    for (int i = size - 1; i >= 0; i--) a.data[i] = 1.0;
}

struct Matrix2 multiply(struct Matrix2 a, struct Matrix2 b)
{
    Matrix2 product;
    set(product, a.rows, b.cols);
    set_to_zero(product);
    
    if (a.cols == b.rows)
    {
        for (int i = 0; i < a.rows; i++)
        {
            for (int j = 0; j < b.cols; j++)
            {
                product.data[i*b.cols + j] = 0.0;
                
                for (int k = 0; k < a.cols; k++)
                {
                    product.data[i*b.cols + j] +=  a.data[i*a.cols + k] * b.data[k * b.cols + j];
                }
            }
        }
    }
    return product;
}

void equal(struct Matrix2 &b, struct Matrix2 a){
    // assuming b is already allocated
    b.rows = a.rows;
    b.cols = a.cols;
    for (int i = 0; i<a.cols*a.rows; i++) b.data[i] = a.data[i];
}

void ludcmp(struct Matrix2 a, int * indx, float *d)
{
    /*
        References
            ----------
            [2] W. H. Press, S. A. Teukolsky, W. T. Vetterling and B. P. Flannery,
                "Numerical Recipes (3rd edition)", Cambridge University Press, 2007,
                page 46. https://www.cec.uchile.cl/cinetica/pcordero/MC_libros/NumericalRecipesinC.pdf
    */
    int n = a.cols;
    int i = 0, imax = 0, j = 0, k = 0;
    float big = 0.0, dum = 0.0, sum = 0.0, temp = 0.0;
    float * vv;
     
    vv = (float*)malloc(n*sizeof(float));
    *d = 1.0;

    for (i=0;i<n;i++){
        big=0.0;
        for (j=0;j<n;j++)
            if ((temp=fabs(a.data[i*a.cols + j])) > big) big=temp;
        //if (big == 0.0) std::cout << "Singular matrix in routine LUDCMP" << std::endl;
        vv[i] = 1.0/big;
    }
 
    
    for (j=0; j<n; j++) {
        for (i=0;i < j;i++) {
            sum = a.data[i*a.cols + j];
            for (k=0;k < i;k++) sum -= a.data[i*a.cols + k]*a.data[k*a.cols + j];
            a.data[i*a.cols + j]=sum;
        }
        big = 0.0;
        for (i = j; i < n;i++) {
            sum = a.data[i*a.cols + j];
            for (k = 0;k < j; k++)
                sum -= a.data[i*a.cols + k]*a.data[k*a.cols + j];
            a.data[i*a.cols + j]=sum;
            if ((dum=vv[i]*fabs(sum)) >= big) {
                big=dum;
                imax=i;
            }
        }
      
                
        if (j != imax) {
            for (k=0;k<n;k++) {
                dum=a.data[imax*a.cols + k];
                a.data[imax*a.cols + k]=a.data[j*a.cols + k];
                a.data[j*a.cols + k]=dum;
            }
            *d = -(*d);
            vv[imax]=vv[j];
        }
                
        indx[j]=imax;
        if (a.data[j*a.cols + j] == 0.0) a.data[j*a.cols + j]= a.tiny;
        if (j != n) {
            dum = 1.0 / (a.data[j*a.cols + j]);
            for (i=j + 1;i<n;i++) a.data[i*a.cols + j] *= dum;
        }
    }
   
    free(vv);
    vv = NULL;
}

void lubksb(struct Matrix2 a, int *indx,float b[])
{
    /*
    References
        ----------
        [2] W. H. Press, S. A. Teukolsky, W. T. Vetterling and B. P. Flannery,
            "Numerical Recipes (3rd edition)", Cambridge University Press, 2007,
            page 46. https://www.cec.uchile.cl/cinetica/pcordero/MC_libros/NumericalRecipesinC.pdf
    */
	int i = 0,ii = 0,ip = 0,j = 0;
	float sum = 0.0;
    int n = a.cols;

	for (i = 0; i < n; i++)
	{
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii)
			for (j=ii;j<=i-1;j++) sum -= a.data[i*a.cols + j]*b[j];
		else if (sum) ii=i;
		b[i]=sum;
	}
	for (i = n - 1; i>=0; i--)
	{
		sum = b[i];
		for (j = i + 1; j < n; j++) sum -= a.data[i*a.cols + j]*b[j];
		b[i] = sum / a.data[i*a.cols + i];
	}
}

struct Matrix2 solve_matrix_eqn(struct Matrix2 a, struct Matrix2 b)
{
    /*
        This is the solution to Ax=b through LU decomposition
        References
            ----------
            .. [1] MATLAB reference documention, "Rank"
                https://www.mathworks.com/help/techdoc/ref/rank.html
            .. [2] W. H. Press, S. A. Teukolsky, W. T. Vetterling and B. P. Flannery,
                "Numerical Recipes (3rd edition)", Cambridge University Press, 2007,
                page 795.

        Modified to start indices at 0
    */
    
    Matrix2 u; 
    set(u, a.rows, b.cols);
    Matrix2 aa;
    set(aa, a.rows, a.cols);
    set_to_zero(aa);
    equal(aa, a);
    
    int m = b.cols;
    
    int n = aa.cols;
    int *indx = (int*)malloc(n*sizeof(int));
    float * col = (float*)malloc(aa.rows * sizeof(float));
    
    for (int j=0; j < m ; j++)
    {
        float d;
        ludcmp(aa, indx, &d);
        for (int i = 0; i < b.rows; i++) col[i] = b.data[i * m + j];
        lubksb(aa, indx, col);
        for (int i = 0; i < aa.rows; i++) u.data[i*m + j] = col[i]; 
    }
    free(indx);
    free(col);
    release(aa);
    return u;
}

struct Matrix2 inverse(struct Matrix2 a)
{
    /*
        References
            ----------
            .. [1] MATLAB reference documention, "Rank"
                https://www.mathworks.com/help/techdoc/ref/rank.html
            .. [2] W. H. Press, S. A. Teukolsky, W. T. Vetterling and B. P. Flannery,
                "Numerical Recipes (3rd edition)", Cambridge University Press, 2007,
                page 795.

        Modified to start indices at 0
    */

    int i, j, *indx;
    float d, *col;
    
    indx = (int*)malloc(a.cols * sizeof(int));
    col = (float*)malloc(a.cols*sizeof(float));
    float * y = (float*)malloc(a.cols*a.rows * sizeof(float));

    Matrix2 inverse;
    set(inverse, a.rows, a.cols);
    equal(inverse, a);

    ludcmp(inverse, indx, &d);
    
    for (j = 0; j < a.cols; j++)
    {
        for (i = 0; i < a.cols; i++) col[i] = 0.0;
        col[j] = 1.0;
        lubksb(inverse, indx, col);

        for (i = 0; i < a.cols; i++) y[i*a.cols + j] = col[i]; 
    }

    for (int i = 0; i < inverse.rows*inverse.cols; i++) inverse.data[i] = y[i];
    free(indx);
    free(col);
    free(y);
    return inverse;
}

struct Matrix2 nr_optimizer(struct Matrix2 jacobian, struct Matrix2 hessian, struct Matrix2 u0)
{
    /*
        Taken from:
            https://github.com/sarahaguasvivas/nlsoft/blob/master/python/controller/soloway_nr.py
    */

    Matrix2 u_optimal;
    int m = u0.cols;
    set(u_optimal, u0.rows, u0.cols);
    equal(u_optimal, u0);
    u_optimal = solve_matrix_eqn(hessian, jacobian);
    return u_optimal;
}
