#include <stdio.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

int func(double t, const double y[], double f[], void *params)
{
	f[0] = y[1];
	f[1] = -y[0];

	return GSL_SUCCESS;
}

int main(void)
{
	gsl_odeiv2_system sys = {func, NULL, 2, NULL};

	// prams: system, driver, initial step size, absolute error, relative error
	gsl_odeiv2_driver *d = gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rk4, 1e-6, 1e-6, 0.0);

	int i;
	double t = 0.0, t1 = 100.0;
	double y[2] = { 1.0, 0.0 };

	for (i = 1; i <= 10000; i++)
	{
		double ti = i * t1 / 10000.0;
		int status = gsl_odeiv2_driver_apply (d, &t, ti, y);

		if (status != GSL_SUCCESS)
		{
			printf ("error, return value=%d\n", status);
			break;
		}

		printf ("%.5e %.5e %.5e\n", t, y[0], y[1]);
	}

	gsl_odeiv2_driver_free (d);
	return 0;
}
