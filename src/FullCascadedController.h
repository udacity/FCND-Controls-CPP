#pragma once

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Simulation/BaseDynamics.h"
#include "Trajectory.h"
#include <iostream>
#include "AttitudeController.h"

class FullCascadedController : public AttitudeController
{
public:
  FullCascadedController(string config) : AttitudeController(config) { Init(); };

  virtual void Init()
	{
    AttitudeController::Init();
    
    //Control2D controllerPlanar;
    ParamsHandle config = SimpleConfig::GetInstance();
    
    // PARAMETERS
    Kp_pos_xy = config->Get(_config+".PGain_Pos_XY", 1.f);
    Kp_pos_z = config->Get(_config + ".PGain_Pos_Z", 2.f);
     
    Kp_vel_xy = config->Get(_config + ".PGain_Vel_XY", 4.f);
    Kp_vel_z = config->Get(_config + ".PGain_Vel_Z", 8.f);
	}

  // returns a desired acceleration in global frame
  V3F PositionControl(V3F position_cmd, V3F velocity_ff, V3F position, V3F velocity, V3F acceleration_ff)
  {
    V3F desiredVel = (position_cmd - position) * V3F(Kp_pos_xy, Kp_pos_xy, Kp_pos_z);
    desiredVel += velocity_ff;
    desiredVel.z = CONSTRAIN(desiredVel.z, -10.f, 3.f);

    // TODO: constrain velocity in other ways?

    // ACCELERATION CONTROL
    V3F desAcc = (desiredVel - velocity) * V3F(Kp_vel_xy, Kp_vel_xy, Kp_vel_z) + acceleration_ff;

    // TODO: params to constrain acceleration?
    desAcc.z = CONSTRAIN(desAcc.z, -10.f, 3.f);
    desAcc.constrain(-8, 8);

    return desAcc;
  }

  virtual VehicleCommand RunControl(float dt, float sim_time)
  {
    curTrajPoint = GetNextTrajectoryPoint(sim_time);    
  
    V3F desAcc = PositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

    return AttitudeControl(desAcc, curTrajPoint.attitude.Yaw());
  }

  // PARAMETERS
  float Kp_pos_xy;
  float Kp_pos_z;

  float Kp_vel_xy;
  float Kp_vel_z;
};
