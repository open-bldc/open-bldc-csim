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

#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#include "dyn_model.h"
#include "misc_utils.h"

#include "controller.h"

bool command_steps_init = false;
struct command_vector command_steps[12][2];

void init_steps(void)
{
	if (!command_steps_init) {

		/* 1b */
		command_steps[0][0].hu=0;
		command_steps[0][0].lu=0;
		command_steps[0][0].hv=0;
		command_steps[0][0].lv=1;
		command_steps[0][0].hw=1;
		command_steps[0][0].lw=0;
		command_steps[0][1].hu=0;
		command_steps[0][1].lu=0;
		command_steps[0][1].hv=0;
		command_steps[0][1].lv=1;
		command_steps[0][1].hw=0;
		command_steps[0][1].lw=0;

		/* 2a */
		command_steps[1][0].hu=1;
		command_steps[1][0].lu=0;
		command_steps[1][0].hv=0;
		command_steps[1][0].lv=1;
		command_steps[1][0].hw=0;
		command_steps[1][0].lw=0;
		command_steps[1][1].hu=0;
		command_steps[1][1].lu=0;
		command_steps[1][1].hv=0;
		command_steps[1][1].lv=1;
		command_steps[1][1].hw=0;
		command_steps[1][1].lw=0;

		/* 2b */
		command_steps[2][0].hu=1;
		command_steps[2][0].lu=0;
		command_steps[2][0].hv=0;
		command_steps[2][0].lv=1;
		command_steps[2][0].hw=0;
		command_steps[2][0].lw=0;
		command_steps[2][1].hu=0;
		command_steps[2][1].lu=0;
		command_steps[2][1].hv=0;
		command_steps[2][1].lv=1;
		command_steps[2][1].hw=0;
		command_steps[2][1].lw=0;

		/* 3a */
		command_steps[3][0].hu=1;
		command_steps[3][0].lu=0;
		command_steps[3][0].hv=0;
		command_steps[3][0].lv=0;
		command_steps[3][0].hw=0;
		command_steps[3][0].lw=1;
		command_steps[3][1].hu=0;
		command_steps[3][1].lu=0;
		command_steps[3][1].hv=0;
		command_steps[3][1].lv=0;
		command_steps[3][1].hw=0;
		command_steps[3][1].lw=1;

		/* 3b */
		command_steps[4][0].hu=1;
		command_steps[4][0].lu=0;
		command_steps[4][0].hv=0;
		command_steps[4][0].lv=0;
		command_steps[4][0].hw=0;
		command_steps[4][0].lw=1;
		command_steps[4][1].hu=0;
		command_steps[4][1].lu=0;
		command_steps[4][1].hv=0;
		command_steps[4][1].lv=0;
		command_steps[4][1].hw=0;
		command_steps[4][1].lw=1;

		/* 4a */
		command_steps[5][0].hu=0;
		command_steps[5][0].lu=0;
		command_steps[5][0].hv=1;
		command_steps[5][0].lv=0;
		command_steps[5][0].hw=0;
		command_steps[5][0].lw=1;
		command_steps[5][1].hu=0;
		command_steps[5][1].lu=0;
		command_steps[5][1].hv=0;
		command_steps[5][1].lv=0;
		command_steps[5][1].hw=0;
		command_steps[5][1].lw=1;

		/* 4b */
		command_steps[6][0].hu=0;
		command_steps[6][0].lu=0;
		command_steps[6][0].hv=1;
		command_steps[6][0].lv=0;
		command_steps[6][0].hw=0;
		command_steps[6][0].lw=1;
		command_steps[6][1].hu=0;
		command_steps[6][1].lu=0;
		command_steps[6][1].hv=0;
		command_steps[6][1].lv=0;
		command_steps[6][1].hw=0;
		command_steps[6][1].lw=1;

		/* 5a */
		command_steps[7][0].hu=0;
		command_steps[7][0].lu=1;
		command_steps[7][0].hv=1;
		command_steps[7][0].lv=0;
		command_steps[7][0].hw=0;
		command_steps[7][0].lw=0;
		command_steps[7][1].hu=0;
		command_steps[7][1].lu=1;
		command_steps[7][1].hv=0;
		command_steps[7][1].lv=0;
		command_steps[7][1].hw=0;
		command_steps[7][1].lw=0;

		/* 5b */
		command_steps[8][0].hu=0;
		command_steps[8][0].lu=1;
		command_steps[8][0].hv=1;
		command_steps[8][0].lv=0;
		command_steps[8][0].hw=0;
		command_steps[8][0].lw=0;
		command_steps[8][1].hu=0;
		command_steps[8][1].lu=1;
		command_steps[8][1].hv=0;
		command_steps[8][1].lv=0;
		command_steps[8][1].hw=0;
		command_steps[8][1].lw=0;

		/* 6a */
		command_steps[9][0].hu=0;
		command_steps[9][0].lu=1;
		command_steps[9][0].hv=0;
		command_steps[9][0].lv=0;
		command_steps[9][0].hw=1;
		command_steps[9][0].lw=0;
		command_steps[9][1].hu=0;
		command_steps[9][1].lu=1;
		command_steps[9][1].hv=0;
		command_steps[9][1].lv=0;
		command_steps[9][1].hw=0;
		command_steps[9][1].lw=0;

		/* 6b */
		command_steps[10][0].hu=0;
		command_steps[10][0].lu=1;
		command_steps[10][0].hv=0;
		command_steps[10][0].lv=0;
		command_steps[10][0].hw=1;
		command_steps[10][0].lw=0;
		command_steps[10][1].hu=0;
		command_steps[10][1].lu=1;
		command_steps[10][1].hv=0;
		command_steps[10][1].lv=0;
		command_steps[10][1].hw=0;
		command_steps[10][1].lw=0;

		/* 1a */
		command_steps[11][0].hu=0;
		command_steps[11][0].lu=0;
		command_steps[11][0].hv=0;
		command_steps[11][0].lv=1;
		command_steps[11][0].hw=1;
		command_steps[11][0].lw=0;
		command_steps[11][1].hu=0;
		command_steps[11][1].lu=0;
		command_steps[11][1].hv=0;
		command_steps[11][1].lv=1;
		command_steps[11][1].hw=0;
		command_steps[11][1].lw=0;
	}
}

int run(double t, const struct setpoint *sp, const struct motor *m, const struct state_vector *sv, struct command_vector *cv)
{
	double pwm_cycle_time = (1. / sp->pwm_frequency);
	double pwm_duty_time = pwm_cycle_time * sp->pwm_duty;
	/* create a virtual encoder */
	double elec_angle = norm_angle(sv->theta * (m->NbPoles/2));
	int slice = angle_slice(elec_angle, 12);
	struct command_vector *selected_cv;

	if (fmod(t, pwm_cycle_time) <= pwm_duty_time) {
		selected_cv = &command_steps[slice][0];
	} else {
		selected_cv = &command_steps[slice][1];
	}

	cv->hu = selected_cv->hu;
	cv->lu = selected_cv->lu;
	cv->hv = selected_cv->hv;
	cv->lv = selected_cv->lv;
	cv->hw = selected_cv->hw;
	cv->lw = selected_cv->lw;

	//printf("step %d ", slice);

	return 1;
}
