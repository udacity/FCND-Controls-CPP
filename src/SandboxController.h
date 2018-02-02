#pragma once

#include "BaseController.h"

// a sandbox controller
// keep your expectations low
// it's only sand

class SandboxController : public BaseController
{
public:
  SandboxController(string config) :BaseController(config) { Init(); };

  virtual void Init()
  {
    BaseController::Init();

    //Control2D controllerPlanar;
    ParamsHandle config = SimpleConfig::GetInstance();

    // we don't do much here
  }

  virtual VehicleCommand RunControl(float dt, float sim_time) 
  {
    // the ordering is Udacity-style: front-left, front-right, back-left, back-right
    //
    // motor commands are in Newtons
    //
    // you can use physical parameters from BaseController such as vehicle mass (M), etc
    // 
    // for example, if you set the motors to M*9.81/4, and the vehicle is flat, and it's an ideal simulation,
    // it should remain in place
    cmd.desiredThrustsN[0] = 4;
    cmd.desiredThrustsN[1] = 3;
    cmd.desiredThrustsN[2] = 2;
    cmd.desiredThrustsN[3] = 1;

    return cmd;
  }

  // PARAMETERS
  float Kp_bank;
  float Kp_yaw;

  V3F Kp_angle_rate;
};