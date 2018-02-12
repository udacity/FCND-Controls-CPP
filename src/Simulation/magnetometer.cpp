#include "magnetometer.h"

void magnetometer::magnetometer_sensor(float declination, SLR::Quaternion<float> attitude, V3F &mag_measurement){
  mag = V3F(0.215212f,0.0f,-0.42741f); // Values taken from WMM 2015, NED Frame, order 10^5 nT // maybe a recheck
  SLR::Quaternion<float> declination_quat;
  declination_quat = SLR::Quaternion<float>::FromEulerYPR(-declination,0.0f,0.0f);
  mag_measurement = declination_quat.Rotate_ItoB(mag);
  mag_measurement = attitude.Rotate_ItoB(mag_measurement);

  std::random_device rd;
  std::mt19937 gen(rd());

  std::normal_distribution<> fx{0,fx_stddev};
  std::normal_distribution<> fy{0,fy_stddev};
  std::normal_distribution<> fz{0,fz_stddev};

  float fx_sample = fx(gen);
  float fy_sample = fy(gen);
  float fz_sample = fz(gen);

  mag_measurement.x += fx_sample;
  mag_measurement.y += fy_sample;
  mag_measurement.z += fz_sample;

}
