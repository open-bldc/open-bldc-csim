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

#ifndef _MISC_UTILS
#define _MISC_UTILS

double rad_of_deg(double d);
double deg_of_rad(double r);
double rpm_of_radps(double rps);
double degps_of_radps(double rps);
double radps_of_rpm(double rpm);
double vpradps_of_rpmpv(double vprpm);
double norm_angle(double alpha);

#endif /* _MISC_UTILS */
