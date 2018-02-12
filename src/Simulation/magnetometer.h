#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <math.h>
#include "Common.h"
#include "Math/Random.h"
#include "Math/Quaternion.h"
#include "matrix/math.hpp"
#include <random>

class magnetometer
{
public:
  V3F mag;
  float fx_stddev = 0.0001f;
  float fy_stddev = 0.0001f;
  float fz_stddev = 0.0001f;
  void magnetometer_sensor(float declination, SLR::Quaternion<float> attitude, V3F &mag_measurement);
};

#endif // MAGNETOMETER_H
