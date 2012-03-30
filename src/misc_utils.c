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
