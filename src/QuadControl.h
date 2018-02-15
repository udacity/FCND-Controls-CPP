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
  V3F HorizontalControl(V3F position_cmd, V3F velocity_ff, V3F position, V3F velocity, V3F acceleration_ff);

  virtual VehicleCommand RunControl(float dt, float sim_time);

  VehicleCommand GenerateMotorCommands(float desCollThrust, V3F desMoment);

  // returns desired yaw rate
  float YawControl(float yaw_cmd, float yaw);

  // returns desired moments
  V3F BodyRateControl(V3F body_rate_cmd, V3F body_rate);

  // returns a desired roll and pitch rate 
  V3F ReducedAttitudeControl(V3F acceleration_cmd, Quaternion<float> attitude, float desCollectiveThrust);

  float AltitudeControl(float desPosZ, float desVelZ, float posZ, float velZ, Quaternion<float> attitude, float accelFF);

  // PARAMETERS
  float Kp_pos_xy, Kp_pos_z;
  float Kp_vel_xy, Kp_vel_z;
  float Kp_bank, Kp_yaw;
  V3F PGain_Omega;
  
  float max_ascent_rate, max_descent_rate;
  float max_speed_xy;
  float max_horiz_accel;
  float max_tilt_angle;

  float min_motor_thrust, max_motor_thrust;
};
