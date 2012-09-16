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
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#include "switch.h"

void switch_set(struct ssn **ssn, double t, bool state)
{
	if (*ssn == NULL) {
		*ssn = malloc(sizeof(struct ssn));
		(*ssn)->next = NULL;
	}

	(*ssn)->time = t;
	(*ssn)->state = state;
	(*ssn)->stop = true;
}

void switch_pwm_gen(struct ssn **ssn, double t, double t1, double freq, double duty, bool inverted)
{
	double cycle_t = (1. / freq);        /* duration of the pwm cycle */
	double duty_t = cycle_t * duty;      /* duration of the duty period */
	struct ssn **cur_ssn = ssn;
	double cur_t = t - fmod(t, cycle_t); /* current time adjusted to the last pwm cycle start */

	/* If duty cycle == 100% we only need one node. */
	if (duty == 1.) {
		switch_set(ssn, t, !inverted);
		return;
	}

	/* If duty cycle == 0% we only need one node too. */
	if (duty == 0.) {
		switch_set(ssn, t, inverted);
		return;
	}

	/* Generate PWM cycle nodes for at least the t to t1 timeframe. */
	while (cur_t < t1) {
		/* Create node for the cycle beginning. */
		switch_set(cur_ssn, cur_t, !inverted);
		cur_ssn = &((*cur_ssn)->next);
		cur_t += duty_t;
		switch_set(cur_ssn, cur_t, inverted);
		cur_ssn = &((*cur_ssn)->next);
		cur_t += cycle_t - duty_t;
	}

	/* Set stop sign. */
	cur_ssn = ssn;
	while ((*cur_ssn)->next && ((*cur_ssn)->time < t1)) {
		(*cur_ssn)->stop = false;
		cur_ssn = &((*cur_ssn)->next);
	}

}

bool switch_get(struct ssn **ssn, double t)
{
	struct ssn **cur_ssn = ssn;
	bool state;

	if (t < (*ssn)->time) {
		/* This should not happen! */
		return -1;
	}

	while (((*cur_ssn)->time <= t)) {
		state = (*cur_ssn)->state;
		if ((*cur_ssn)->stop) {
			break;
		}
		cur_ssn = &((*cur_ssn)->next);
	}

	return state;
}

void switch_clear(struct ssn **ssn)
{
	struct ssn **next_ssn;

	while (*ssn) {
		next_ssn = &((*ssn)->next);
		free(*ssn);
		*ssn = NULL;
		ssn = next_ssn;
	}
}
