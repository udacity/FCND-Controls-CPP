#pragma once

#include "BaseController.h"

// a simple controller that tries to bring the vehicle to 0 attitude
class AttitudeController : public BaseController
{
public:
  AttitudeController(string config) :BaseController(config) { Init(); };

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

  VehicleCommand GenerateMotorCommands(float desCollThrust, V3F desMoment)
  {
    // Convert desired moment into differential thrusts
    V3F diffThrust;

    // for X shaped quad
    diffThrust.x = desMoment.x / L / 2.f / sqrtf(2);
    diffThrust.y = desMoment.y / L / 2.f / sqrtf(2);
    diffThrust.z = desMoment.z / 4.f / kappa;

    // MIXING
    // combine the collective thrust with the differential thrust commands to find desired motor thrusts
    // X Shaped Quad (NED Frame)
    cmd.desiredThrustsN[0] = desCollThrust / 4.f - diffThrust.z + diffThrust.y + diffThrust.x; // front left
    cmd.desiredThrustsN[1] = desCollThrust / 4.f + diffThrust.z + diffThrust.y - diffThrust.x; // front right
    cmd.desiredThrustsN[2] = desCollThrust / 4.f + diffThrust.z - diffThrust.y + diffThrust.x; // rear left
    cmd.desiredThrustsN[3] = desCollThrust / 4.f - diffThrust.z - diffThrust.y - diffThrust.x; // rear right

    return cmd;
  }

  // returns desired yaw rate
  float YawControl(float yaw_cmd, float yaw)
  {
    float yaw_error = yaw_cmd - yaw;
    yaw_error = fmodf(yaw_error, F_PI*2.f);
    if (yaw_error > F_PI)
    {
      yaw_error -= 2.f * F_PI;
    }
    else if (yaw_error < -F_PI)
    {
      yaw_error += 2.f * F_PI;
    }
    return yaw_error * Kp_yaw;
  }

  // returns desired moments
  V3F BodyRateControl(V3F body_rate_cmd, V3F body_rate)
  {
    V3F rate_error = body_rate_cmd - body_rate;
    V3F omega_dot_des = rate_error * Kp_angle_rate;
    V3F moment_cmd = omega_dot_des * V3F(Ixx, Iyy, Izz);;
    return moment_cmd;
  }

  // returns a desired roll and pitch rate 
  V3F LateralControl(V3F acceleration_cmd, Quaternion<float> attitude)
  {
    // get our current Euler angles
    float curRoll, curPitch, curYaw;
    attitude.ToEulerYPR(curRoll, curPitch, curYaw);

    // Simplistic thrust vector + desired bank angle calc
    V3F thrustVector = M*acceleration_cmd + V3F(0, 0, -M*9.81f);

    V3F thrustVectorRotated = thrustVector;
    thrustVectorRotated.x = thrustVector.x*cos(curYaw) + thrustVector.y*sin(curYaw);
    thrustVectorRotated.y = thrustVector.y*cos(curYaw) - thrustVector.x*sin(curYaw);

    float desBankPitch = atan2f(-thrustVectorRotated.x, -thrustVector.z);
    float desBankRoll = atan2f(thrustVectorRotated.y, -thrustVector.z);
    
    // ATTITUDE CONTROL -- can be improved: we're mixing Euler & bank angles    
    V3F desOmega;
    desOmega.x = (desBankRoll - curRoll) * Kp_bank;
    desOmega.y = (desBankPitch - curPitch) * Kp_bank;

    return desOmega;
  }

  VehicleCommand AttitudeControl(V3F des_acceleration, float yaw_cmd)
  {
    // Simplistic thrust vector + desired bank angle calc
    V3F thrustVector = M*des_acceleration + V3F(0, 0, -M*9.81f);
    float desCollThrust = thrustVector.mag();

    V3F desOmega = LateralControl(des_acceleration, estAtt);
    desOmega.z = YawControl(yaw_cmd, estAtt.Yaw());

    V3F desMoment = BodyRateControl(desOmega, estOmega);

    return GenerateMotorCommands(desCollThrust, desMoment);
  }

  // on its own, just go to 0-attitude
  virtual VehicleCommand RunControl(float dt, float sim_time) 
  {
    return AttitudeControl(V3F(0, 0, 0), 0);
  }

  // PARAMETERS
  float Kp_bank;
  float Kp_yaw;

  V3F Kp_angle_rate;
};