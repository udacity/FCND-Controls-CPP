#pragma once

#include "Math/V3F.h"
#include "Math/Quaternion.h"

using SLR::Quaternion;

struct GlobalPose
{
  V3F pos;
  Quaternion<float> q;
};

struct VehicleCommand
{
  VehicleCommand()
  {
    mode = 0;
    desiredThrustsN[0] = desiredThrustsN[1] = desiredThrustsN[2] = desiredThrustsN[3] = 0;
  }
  float desiredThrustsN[4]; // N, motor A, motor B, motor C, motor D
  uint8_t mode;
};

// Struct for holding all of the data related to a single trajectory point
struct TrajectoryPoint {
  float time;
  V3F position;
  V3F velocity;
  V3F omega;
  V3F accel;
  Quaternion<float> attitude;

  // Initialise all fields to zero when declared
  TrajectoryPoint() :
    time(0.f),
    position(0.f, 0.f, 0.f),
    velocity(0.f, 0.f, 0.f),
    omega(0.f, 0.f, 0.f),
    attitude(0.f, 0.f, 0.f, 0.f)
  {
  }
};