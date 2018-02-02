#pragma once

#include "BaseController.h"

// a simple controller that tries to bring the vehicle to 0 attitude
class SimpleAttitudeController : public BaseController
{
public:
  SimpleAttitudeController(string config) :BaseController(config) { Init(); };

  virtual void Init()
  {
    BaseController::Init();

    //Control2D controllerPlanar;
    ParamsHandle config = SimpleConfig::GetInstance();

    // PARAMETERS
    Kp_bank = config->Get("Controller.PGain_Bank", 25.f); 
    Kp_yaw = config->Get("Controller.PGain_Yaw", 10.f);

    Kp_angle_rate = config->Get("Controller.PGain_Omega", V3F(100.f, 100.f, 10.f));
  }

  virtual VehicleCommand RunControl(float dt, float sim_time) 
  {
    // assume our reference is 0-attitude!
    float desYaw = 0;
    float desBankRoll = 0;
    float desBankPitch = 0;

    // get our current Euler angles
    float curRoll, curPitch, curYaw;
    estAtt.ToEulerYPR(curRoll, curPitch, curYaw);

    // ATTITUDE CONTROL -- can be improved: we're mixing Euler & bank angles    
    desOmega.z = (desYaw - curYaw) * Kp_yaw;
    desOmega.x = (desBankRoll - curRoll) * Kp_bank;
    desOmega.y = (desBankPitch - curPitch) * Kp_bank;

    // ANGLE RATE CONTROL
    desOmegaDot = (desOmega - estOmega) * Kp_angle_rate;
    V3F desMoment = desOmegaDot * V3F(Ixx, Iyy, Izz);

    // Convert desired moment into differential thrusts
    V3F diffThrust;

    // for X shaped quad
    diffThrust.x = desMoment.x / L / 2.f / sqrtf(2);
    diffThrust.y = desMoment.y / L / 2.f / sqrtf(2);
    diffThrust.z = desMoment.z / 4.f / kappa;

    // set collective thrust to nominal hover value
    desCollThrust = M * 9.81f;

    // MIXING
    // combine the collective thrust with the differential thrust commands to find desired motor thrusts
    // X Shaped Quad (NED Frame)
    cmd.desiredThrustsN[0] = desCollThrust / 4.f - diffThrust.z + diffThrust.y + diffThrust.x; // front left
    cmd.desiredThrustsN[1] = desCollThrust / 4.f + diffThrust.z + diffThrust.y - diffThrust.x; // front right
    cmd.desiredThrustsN[2] = desCollThrust / 4.f + diffThrust.z - diffThrust.y + diffThrust.x; // rear left
    cmd.desiredThrustsN[3] = desCollThrust / 4.f - diffThrust.z - diffThrust.y - diffThrust.x; // rear right

    return cmd;
  }

  // PARAMETERS
  float Kp_bank;
  float Kp_yaw;

  V3F Kp_angle_rate;
};