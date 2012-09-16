/*
 * Open-BLDC CSim - Open BrushLess DC Motor Controller C Simulator
 * Copyright (C) 2012 by Piotr Esden-Tempski <piotr@esden.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

struct variable_vector {
	double x1;
	double x2;
};

int func(double t, const double y[], double f[], void *params)
{
	struct variable_vector *ivv = (struct variable_vector *)y;
	struct variable_vector *ovv = (struct variable_vector *)f;
	ovv->x1 = ivv->x2;
	ovv->x2 = -ivv->x1;

	return GSL_SUCCESS;
}

int main(void)
{
	gsl_odeiv2_system sys = {func, NULL, 2, NULL};

	// prams: system, driver, initial step size, absolute error, relative error
	gsl_odeiv2_driver *d = gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rk4, 1e-6, 1e-6, 0.0);

	int i;
	double t = 0.0, t1 = 100.0;
	struct variable_vector *vv;
	
	vv = malloc(sizeof(struct variable_vector));
	
	vv->x1 = 1.0;
	vv->x2 = 0.0;

	for (i = 1; i <= 10000; i++)
	{
		double ti = i * t1 / 10000.0;
		int status = gsl_odeiv2_driver_apply (d, &t, ti, (double *)vv);

		if (status != GSL_SUCCESS)
		{
			printf ("error, return value=%d\n", status);
			break;
		}

		printf ("%.5e %.5e %.5e\n", t, vv->x1, vv->x2);
	}

	gsl_odeiv2_driver_free (d);

	free(vv);

	return 0;
}
