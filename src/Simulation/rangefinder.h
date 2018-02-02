#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "Common.h"
#include "Math/Random.h"
#include "Math/Quaternion.h"
#include "matrix/math.hpp"
#include <random>

class rangefinder
{
public:
  float fd_stddev = 0.0001f;
  void range_sensor(V3F position, SLR::Quaternion<float> attitude, float &measurement);
};

#endif // RANGEFINDER_H
