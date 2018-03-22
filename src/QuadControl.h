#pragma once


#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"

class QuadControl : public BaseController
{
public:
  QuadControl(string config) : BaseController(config) { Init(); };

  virtual void Init();

  // returns a desired acceleration in global frame
  V3F LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd);

  virtual VehicleCommand RunControl(float dt, float sim_time);

  VehicleCommand GenerateMotorCommands(float collThrustCmd, V3F momentCmd);

  // returns desired yaw rate
  float YawControl(float yawCmd, float yaw);

  // returns desired moments
  V3F BodyRateControl(V3F pqrCmd, V3F pqr);

  // returns a desired roll and pitch rate 
  V3F RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd);

  float AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt);

  // -------------- PARAMETERS --------------

  // controller gains
  float kpPosXY, kpPosZ;
  float kpVelXY, kpVelZ;
  float kpBank, kpYaw;
  float KiPosZ;
  V3F kpPQR;
  
  // limits & saturations
  float maxAscentRate, maxDescentRate;
  float maxSpeedXY;
  float maxAccelXY;
  float maxTiltAngle;
  float minMotorThrust, maxMotorThrust;

  // integral control
  float integratedAltitudeError;
};
