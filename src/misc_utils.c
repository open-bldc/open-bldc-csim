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

#include <math.h>

#include "misc_utils.h"


double rad_of_deg(double d)
{
  return ((d/180.) * M_PI);
}

double deg_of_rad(double r)
{
  return ((r*180.) / M_PI);
}

double rpm_of_radps(double rps)
{
  return (rps / (2 * M_PI) * 60);
}

double degps_of_radps(double rps)
{
  return (rps / (2 * M_PI) * 60 * 360);
}

double radps_of_rpm(double rpm)
{
  return (rpm * (2 * M_PI) / 60);
}

double vpradps_of_rpmpv(double vprpm)
{
  return (30 / (vprpm * M_PI));
}

double norm_angle(double alpha)
{
  double alpha_n = fmod(alpha, M_PI * 2);

  if (alpha_n < 0.) {
    alpha_n += (M_PI * 2);
  }

  return alpha_n;
}
