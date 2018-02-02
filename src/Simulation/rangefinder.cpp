#include "rangefinder.h"

// Assumption: The distance sensor is located at the centre of the crazyflie, implying that the yaw motion does not affect the measurements

void rangefinder::range_sensor(V3F position, SLR::Quaternion<float> attitude, float &measurement){

  V3D ypr = attitude.ToEulerYPR();
  measurement = position.z/(cos(ypr.y)*cos(ypr.z));

  // Now adding the normal noise
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> fd{0, fd_stddev};

  float fd_sample = fd(gen);

  measurement += fd_sample;
}
