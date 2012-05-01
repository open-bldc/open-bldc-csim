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

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#if defined(__cplusplus)
extern "C" {
#endif

struct setpoint {
  double pwm_frequency;
  double pwm_duty;
};

int run(double t, const struct setpoint *sp, const struct motor *m, const struct state_vector *sv, struct command_vector *cv);

#if defined(__cplusplus)
} /* extern "C" */
#endif

#endif /* _CONTROLLER_H */
