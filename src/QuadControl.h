#pragma once

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Simulation/BaseDynamics.h"
#include "Trajectory.h"
#include "BaseController.h"

class QuadControl : public BaseController
{
public:
  QuadControl(string config) : BaseController(config) { Init(); };

  virtual void Init();

  // returns a desired acceleration in global frame
  V3F PositionControl(V3F position_cmd, V3F velocity_ff, V3F position, V3F velocity, V3F acceleration_ff);

  virtual VehicleCommand RunControl(float dt, float sim_time);

  VehicleCommand GenerateMotorCommands(float desCollThrust, V3F desMoment);

  // returns desired yaw rate
  float YawControl(float yaw_cmd, float yaw);

  // returns desired moments
  V3F BodyRateControl(V3F body_rate_cmd, V3F body_rate);

  // returns a desired roll and pitch rate 
  V3F LateralControl(V3F acceleration_cmd, Quaternion<float> attitude);

  VehicleCommand AttitudeControl(V3F des_acceleration, float yaw_cmd);

  // PARAMETERS
  float Kp_pos_xy, Kp_pos_z;
  float Kp_vel_xy, Kp_vel_z;
  float Kp_bank, Kp_yaw;
  V3F Kp_angle_rate;
};
