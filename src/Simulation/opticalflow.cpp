#include "opticalflow.h"

void opticalflow::opticalflow_sensor(float dt, V3F position, V3F velocity, SLR::Quaternion<float> attitude, V3F omega, float &x_measurement, float &y_measurement){
  x_measurement = (dt*N_pixel/aperture) * ((attitude.Rotate_ItoB(velocity).x / attitude.Rotate_ItoB(position).z) + omega_factor * omega.y);
  y_measurement = (dt*N_pixel/aperture) * ((attitude.Rotate_ItoB(velocity).y / attitude.Rotate_ItoB(position).z) - omega_factor * omega.x);

  // The signs used for omega above need to be checked again - not sure which frame is used by crazyflie

  std::random_device rd;
  std::mt19937 gen(rd());

  std::normal_distribution<> fx{0,fx_stddev};
  std::normal_distribution<> fy{0,fy_stddev};

  float fx_sample = fx(gen);
  float fy_sample = fy(gen);

  x_measurement += fx_sample;
  y_measurement += fy_sample;

}
