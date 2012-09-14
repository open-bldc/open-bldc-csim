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
#include <QtCore>

#include "dyn_model.h"
#include "controller.h"

struct motor motor = {
	motor.inertia = 1.1,
	motor.damping = 0.001,
	motor.static_friction = 0.1,
	motor.Kv = 1./32.3*1000,
	motor.L = 0.00207,
	motor.M = -0.00069,
	motor.R = 11.9,
	motor.VDC = 100,
	motor.NbPoles = 4
};

struct command_vector cv = {
	cv.hu = true,
	cv.lu = false,
	cv.hv = false,
	cv.lv = true,
	cv.hw = false,
	cv.lw = false
};

struct perturbation_vector pv = {
	pv.torque = 0.2
};

struct parameters params = {
	params.m = &motor,
	params.cv = &cv,
	params.pv = &pv
};

struct setpoint setpoint = {
        setpoint.pwm_frequency = 16000,
        setpoint.pwm_duty = 1
};

int main(void)
{

	gsl_odeiv2_system sys = {dyn, NULL, 5, &params};

	// prams: system, driver, initial step size, absolute error, relative error
	gsl_odeiv2_driver *d = gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rk4, 1e-6, 1e-6, 0.0);

	int i;
	double t = 0.0, t1 = 2.0;
	double sim_freq = 100000;
	int steps = (t1-t) * sim_freq;
	struct state_vector sv;
	double perc;

	QFile file("obldc-csim-out.dat");
	file.open(QIODevice::WriteOnly);
	QDataStream out(&file);   // we will serialize the data into the file

	init_state(&sv);
	run(t, &setpoint, &motor, &sv, &cv);

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
		out << t << sv.iu << sv.iv << sv.iw << sv.theta << sv.omega;
		//fprintf (stderr, "%.5e %.5e %.5e %.5e %.5e %.5e\n", t, sv.iu, sv.iv, sv.iw, sv.theta, sv.omega);

		run(t, &setpoint, &motor, &sv, &cv);

	}

	gsl_odeiv2_driver_free (d);
	file.close();
	return 0;
}
