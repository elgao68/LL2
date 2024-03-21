////////////////////////////////////////////////////////////////////////////                                                                        
// utils.c                                                              
// Description:		definitions file,
//					assorted utility functions                                
//                                                    
// Gabriel Aguirre-Ollinger
// Start date: 02/21/2005                                                                        
////////////////////////////////////////////////////////////////////////////

#include <_math_utils.h>

int
round_dbl(double value) {
	int sign;
	double fraction;

	if (value >= 0)
		sign = 1;
	else
		sign = -1;

	fraction = fabs(value) - floor(fabs(value));
	if (fraction < 0.5)
		return (int)(sign*floor(fabs(value)));
	else
		return (int)(sign*ceil(fabs(value)));
}

double
norm2(double* vect, int dim) {
	double norm_sqrd = 0;
	int vi;
	for (vi = 0; vi < dim; vi++)
		norm_sqrd += vect[vi]*vect[vi];

	return sqrt(norm_sqrd);
}

double
int_trapezoidal(double* x, double y_init, double n, double T_INC) {
	// n: number of time intervals
	double	y0, y1 = 0;
	int		ti;
	y0 = y_init;

	for (ti = 1; ti <= n; ti++) {
		y1 = y0 + 1/2.0*(x[ti - 1] + x[ti])*T_INC;
		y0 = y1;
	}

	return y1;
}

double
normald_angle(double ang) {
	return atan2(sin(ang), cos(ang));
}

int	
sgn(double arg) {
	if (arg > 0)
		return 1;
	else if (arg < 0)
		return -1;
	else 
		return 0;
}

double
std_dev_moving_static(double x_in, double* mean_x, const int num_samples) {
	// Standard deviation and mean of a moving-window sample:

	static double*	x; // input samples arrays
	static int		initialize = 1;

	double			var_x; // variance of the samples
	int i;

	////////////////////////////////////////////////////
	// Initialize input data array:
	////////////////////////////////////////////////////

	if (initialize) {
		x = (double*)calloc(num_samples, sizeof(double));
		initialize = 0;
	}
	
	////////////////////////////////////////////////////
	// Compute mean: 
	////////////////////////////////////////////////////

	x[0] = x_in; // current input signal value	

	*mean_x = 0;
	for (i = 0; i < num_samples; i++)
		*mean_x += x[i]/num_samples;
		
	////////////////////////////////////////////////////
	// Compute variance: 
	////////////////////////////////////////////////////

	var_x = 0;
	for (i = 0; i < num_samples; i++)
		var_x += (x[i] - *mean_x)*(x[i] - *mean_x)/num_samples;

	///////////////////////////////////////////////////	
	// Update data array:
	///////////////////////////////////////////////////	

	for (i = num_samples - 1; i >= 1; i--)
		x[i] = x[i - 1];

	return sqrt(var_x);
}

double 
cubic_interpolation(double* y, double u) {
   double a0, a1, a2, a3;

   a0 = y[3] - y[2] - y[0] + y[1];
   a1 = y[0] - y[1] - a0;
   a2 = y[2] - y[0];
   a3 = y[1];

   return a0*u*u*u + a1*u*u + a2*u + a3;
}

double	
normal_pdf(double x, double mean, double stdev) {
	return 1.0/stdev/sqrt(2*PI)*exp(-(x - mean)*(x - mean)/2/stdev/stdev);
}

double
t_integrator_ode(double x, param_struct params) {
	double dt_x_m = params.p_dbl[0];	
	return dt_x_m;
}

int
factorial(int n) {
	int factor = 1;
	int i;

	for (i = 1; i <= n; i++)
		factor = factor*i;

	return factor;
}

double 
vel_cubic_interpolation(double* y, double u, double dt_u) {
   double a0, a1, a2, a3;

   a0 = y[3] - y[2] - y[0] + y[1];
   a1 = y[0] - y[1] - a0;
   a2 = y[2] - y[0];
   a3 = y[1];

   return (3*a0*u*u + 2*a1*u + a2)*dt_u;
}

////////////////////////////////////////////////////////////////////////
// Numerical methods
////////////////////////////////////////////////////////////////////////

void	
solve_ode_runge_kutta_sys1_ord4(double* x, double T, double (*ode_func)(double, param_struct), param_struct params) {

	double k1 = (*ode_func)(*x, params);	
	double k2 = (*ode_func)(*x + T/2.0*k1, params);
	double k3 = (*ode_func)(*x + T/2.0*k2, params);
	double k4 = (*ode_func)(*x + T*k3, params);

	*x = *x + T/6.0*(k1 + 2*k2 + 2*k3 + k4);
}

void	
solve_ode_runge_kutta_sys2_ord4(double* x, double *dt_x, double T, double (*ode_func)(double, double, param_struct), param_struct params) {

	double k1 = (*ode_func)(*x, *dt_x, params);	
	double k2 = (*ode_func)(*x + T/2.0*(*dt_x) + T*T/8.0*k1, *dt_x + T/2.0*k1, params);
	double k3 = (*ode_func)(*x + T/2.0*(*dt_x) + T*T/8.0*k2, *dt_x + T/2.0*k2, params);
	double k4 = (*ode_func)(*x + T*(*dt_x)     + T*T/2.0*k3, *dt_x + T*k3,     params);

	*x =   *x + T*(*dt_x) + T*T/6.0*(k1 + k2 + k3);
	*dt_x =        *dt_x  +   T/6.0*(k1 + 2*k2 + 2*k3 + k4);
}

void	
solve_ode_pred_corr_sys2_ord2_static(double* x_0, double* dt_x_0, double T, 
	double (*f)(double, double, param_struct), param_struct par) {
		
	static double	x[3];
	static double	dt_x[3];
	double 			x_next, dt_x_next;// values at t + 1
	double 			x_next_pr, dt_x_next_pr, f_next;
	static int 		si = ORD_2; // initial time step = -2

	if (si == 0) {
		x[0] = *x_0;
		dt_x[0] = *dt_x_0;

		// Predictor step:
		x_next_pr = x[0] + T*dt_x[0] + T*T/24.0*(
			19*(*f)(x[0], dt_x[0], par) 
				-10*(*f)(x[1], dt_x[1], par) 
					+3*(*f)(x[2], dt_x[2], par));

		dt_x_next_pr = 1/T*(x_next_pr - x[0]) + T/24.0*(
			27*(*f)(x[0], dt_x[0], par) 
				-22*(*f)(x[1], dt_x[1], par) 
					+7*(*f)(x[2], dt_x[2], par));

		// ODE function at next step:
		f_next = (*f)(x_next_pr, dt_x_next_pr, par);

		// Correction step:
		x_next = x[0] + T*dt_x[0] + T*T/24.0*(
			3*f_next 
				+10*(*f)(x[0], dt_x[0], par) 
					-(*f)(x[1], dt_x[1], par));
		
		dt_x_next = 1/T*(x_next - x[0]) + T/24.0*(
			7*f_next 
				+6*(*f)(x[0], dt_x[0], par) 
					-(*f)(x[1], dt_x[1], par));
		
		// Recursion step:
		x[2] = x[1];
		dt_x[2] = dt_x[1];
		x[1] = x[0];
		dt_x[1] = dt_x[0];
		*x_0 = x[0] = x_next; 
		*dt_x_0 = dt_x[0] = dt_x_next;
	}
	else {
		// Use Runge-Kutta to fill up initial steps:
		if (si < ORD_2)
			solve_ode_runge_kutta_sys2_ord4(x_0, dt_x_0, T, f, par);

		x[si] = *x_0;
		dt_x[si] = *dt_x_0;

		if (si == 1) {
			x[si + 1] = x[si];
			dt_x[si + 1] = dt_x[si];
		}
		si--;
	}
}

////////////////////////////////////////////////////////////////////////
// Matrix operations taken from "Numerical recipes in C", Ch. 2.3
////////////////////////////////////////////////////////////////////////

double*
vector(int nl, int nh) {
	double* v;

	v = (double*)malloc((nh-nl+1+NR_END)*sizeof(double));
	return (v - nl + NR_END);
}

int*
ivector(int nl, int nh) {
	int* v;

	v = (int*)malloc((nh-nl+1+NR_END)*sizeof(int));
	return (v - nl + NR_END);
}

void	
free_vector(double* v, int nl, int nh) {
	free((FREE_ARG)(v + nl - NR_END));
}

void	
lu_decomp(double **a, int n, int *indx, double *d) {
	// LU decomposition: taken from "Numerical recipes in C", Ch. 2.3
	int i, imax = 0, j, k;
	double big, dum, sum, temp;
	double *vv;

	vv = vector(1,n);
	*d = 1.0;

	for (i = 1; i <= n; i++) {
		big = 0.0;

		for (j = 1; j <= n; j++) 
			if ((temp = fabs(a[i][j])) > big)
				big = temp;
		vv[i] = 1.0/big;
	}

	for (j = 1; j <= n; j++) {

		for (i = 1; i < j; i++) {
			sum = a[i][j];
			for (k = 1; k < i; k++) 
				sum -= a[i][k]*a[k][j];
			a[i][j] = sum;
		}

		big = 0.0;

		for (i = j; i <= n; i++) {
			sum = a[i][j];
			for (k = 1; k < j; k++) 
				sum -= a[i][k]*a[k][j];
			
			a[i][j] = sum;
			if ((dum = vv[i]*fabs(sum)) >= big) {
				big = dum;
				imax = i;
			}
		}

		if (j != imax) {
			for (k = 1; k <= n; k++) {
				dum = a[imax][k];
				a[imax][k] = a[j][k];
				a[j][k] = dum;
			}
			*d = -(*d);
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		// if (a[j][j] == 0.0) 
		//	a[j][j] = TINY;

		if (j != n)	{
			dum = 1.0/a[j][j];
			for (i = j + 1; i <= n; i++)
				 a[i][j] *= dum;
		}
	}

	free_vector(vv, 1, n);
}

void	
lu_backsub(double **a, int n, int *indx, double* b) {
	int	i, ii = 0, ip, j;
	double sum;

	for (i = 1; i <= n; i++) {
		ip = indx[i];
		sum = b[ip];
		b[ip] = b[i];
		if (ii)
			for (j = ii; j <= i - 1; j++) 
				sum -= a[i][j]*b[j];
		else if (sum)
			ii = i;
		b[i] = sum;
	}

	for (i = n; i >= 1; i--) {
		sum = b[i];
		for (j = i + 1; j <= n; j++) 
			sum -= a[i][j]*b[j];

		b[i] = sum/a[i][i];
	}
}

void	
matr_inv_lu(double **a, int n) {
	double **y, d, *col;
	int i, j, *indx;
	int ri;

	indx = (int*)malloc((n + 1)*sizeof(int));
	col = (double*)malloc((n + 1)*sizeof(double));
	
	y = (double**)malloc(n*sizeof(double*));
	for (ri = 0; ri < n + 1; ri++)
		y[ri] = (double*)malloc((n + 1)*sizeof(double));

	lu_decomp(a, n, indx, &d);

	for (j = 1; j <= n; j++) {
		for (i = 1; i <= n; i++)
			col[i] = 0.0;
		col[j] = 1.0;
		lu_backsub(a, n, indx, col);
		
		for (i = 1; i <= n; i++)
			y[i][j] = col[i];
	}

	for (i = 1; i <= n; i++)
		for (j = 1; j <= n; j++) 
		           a[i][j] = y[i][j];
}

void
gauss_jordan_pivot(double **a, int n, double **b, int m) {
	int		*indxc, *indxr, *ipiv;
	int		i, icol = 0, irow = 0, j, k, l, ll;
	double	big, dum, pivinv, temp;

	indxc = ivector(1,n);
	indxr = ivector(1,n);
	ipiv = ivector(1,n);

	for (j = 1; j <= n; j++) 
		ipiv[j] = 0;

	for (i = 1; i<=n; i++) {
		big = 0.0;
		for (j = 1; j<=n; j++)
			if (ipiv[j] != 1)
				for (k = 1; k<=n; k++) {
					if (ipiv[k] == 0) {
						if (fabs(a[j][k]) >= big) {
							big = fabs(a[j][k]);
							irow = j;
							icol = k;
						}
					}  
				}
		
		++(ipiv[icol]);

		if (irow != icol) {
			for (l = 1; l<=n; l++) {
				temp = a[irow][l];
				a[irow][l] = a[icol][l];
				a[icol][l] = temp;
			}
			for (l = 1; l<=m; l++) {
				temp = b[irow][l];
				b[irow][l] = b[icol][l];
				b[icol][l] = temp;
			}
		}
		indxr[i] = irow;
		indxc[i] = icol;
		
		pivinv = 1.0 / a[icol][icol];
		a[icol][icol] = 1.0;

		for (l = 1; l<=n; l++) 
			a[icol][l] *= pivinv;
		for (l = 1; l<=m; l++) 
			b[icol][l] *= pivinv;

		for (ll = 1; ll<=n; ll++)
			if (ll != icol) {
				dum = a[ll][icol];
				a[ll][icol] = 0.0;
				for (l = 1; l<=n; l++) 
					a[ll][l] -= a[icol][l]*dum;
				for (l = 1; l<=m; l++) 
					b[ll][l] -= b[icol][l]*dum;
			}
	}

	for (l = n; l>=1; l--) {
		if (indxr[l] != indxc[l])
			for (k = 1; k<=n; k++) {
				temp = a[k][indxr[l]];
				a[k][indxr[l]] = a[k][indxc[l]];
				a[k][indxc[l]] = temp;
			}
	}
}


