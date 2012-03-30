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
#include <stdbool.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "dyn_model.h"

struct motor motor = {
	.inertia = 0.000007,
	.damping = 0.001166,
	.static_friction = 0.1,
	.Kv = 1./32.3*1000,
	.L = 0.00207,
	.M = -0.00069,
	.R = 11.9,
	.VDC = 300,
	.NbPoles = 4
};

struct command_vector cv = {
	.hu = true,
	.lu = false,
	.hv = false,
	.lv = true,
	.hw = false,
	.lw = false,
};

struct perturbation_vector pv = {
	.torque = 0.
};

struct parameters params = {
	.m = &motor,
	.cv = &cv,
	.pv = &pv
};

int main(void)
{

	gsl_odeiv2_system sys = {dyn, NULL, 5, &params};

	// prams: system, driver, initial step size, absolute error, relative error
	gsl_odeiv2_driver *d = gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rk4, 1e-6, 1e-6, 0.0);

	int i;
	double t = 0.0, t1 = 1.0;
	double sim_freq = 1000000;
	int steps = (t1-t) * sim_freq;
	struct state_vector sv;
	double perc;

	init_state(&sv);

	for (i = 1; i <= steps; i++)
	{
		double ti = i * (t1 / steps);
		int status = gsl_odeiv2_driver_apply (d, &t, ti, (double *)&sv);

		if (status != GSL_SUCCESS)
		{
			printf ("error, return value=%d\n", status);
			break;
		}

		perc = (100./steps)*i;
		printf("Progress: %.2lf%%\r", perc);
		fprintf (stderr, "%.5e %.5e %.5e %.5e %.5e %.5e\n", t, sv.iu, sv.iv, sv.iw, sv.theta, sv.omega);
	}

	gsl_odeiv2_driver_free (d);
	return 0;
}
