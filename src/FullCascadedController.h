#pragma once

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Simulation/BaseDynamics.h"
#include "Trajectory.h"
#include <iostream>
#include "BaseController.h"

class FullCascadedController : public BaseController
{
public:
  FullCascadedController(string config) :BaseController(config) { Init(); };

  virtual void Init()
	{
    BaseController::Init();
    
    //Control2D controllerPlanar;
    ParamsHandle config = SimpleConfig::GetInstance();
    
    // PARAMETERS
    Kp_pos_xy = config->Get(_config+".PGain_Pos_XY", 1.f);
    Kp_pos_z = config->Get(_config + ".PGain_Pos_Z", 2.f);
     
    Kp_vel_xy = config->Get(_config + ".PGain_Vel_XY", 4.f);
    Kp_vel_z = config->Get(_config + ".PGain_Vel_Z", 8.f);

    Kp_bank = config->Get(_config + ".PGain_Bank", 25.f);
    Kp_yaw = config->Get(_config + ".PGain_Yaw", 10.f);

    Kp_angle_rate = config->Get(_config + ".PGain_Omega", V3F(100.f, 100.f, 10.f));
	}

  VehicleCommand RunControl(float dt, float sim_time)
  {
    TrajectoryPoint tpt = GetNextTrajectoryPoint(sim_time);
    desiredPos = tpt.position;
  
    // VELOCITY CONTROL
    V3F desiredVel = (desiredPos - estPos) * V3F(Kp_pos_xy, Kp_pos_xy, Kp_pos_z);

    // If we need the motion to be constrained in a plane (XZ in this simulation), we also need the controller to ignore the y commands and hence:
    //if (config->Get("Sim.Quad.FlightMode","Full3D") == "PlanarXZ")
    //{
    //  desiredVel.y = 0.f;
    //}
    desiredVel.z = CONSTRAIN(desiredVel.z, -3.f, 4.f);

    // ACCELERATION CONTROL
    V3F desAcc = (desiredVel - estVel) * V3F(Kp_vel_xy, Kp_vel_xy, Kp_vel_z);


    // cap the acceleration
    desAcc.z = CONSTRAIN(desAcc.z, -3.f, 6.f);
    desAcc.constrain(-8, 8);

    // Simplistic thrust vector + desired bank angle calc
    V3F thrustVector = M*desAcc + V3F(0, 0, -M*9.81f);

    desCollThrust = thrustVector.mag();
    float curRoll, curPitch, curYaw;
    estAtt.ToEulerYPR(curRoll, curPitch, curYaw);

    //V3F thrustBodyFrame = estAtt.Rotate_ItoB(thrustVector);
    V3F thrustVectorRotated = thrustVector;
    
    thrustVectorRotated.x = thrustVector.x*cos(curYaw) + thrustVector.y*sin(curYaw);
    thrustVectorRotated.y = thrustVector.y*cos(curYaw) - thrustVector.x*sin(curYaw);
    float desBankPitch = atan2f(-thrustVectorRotated.x, -thrustVector.z);
    float desBankRoll = atan2f(thrustVectorRotated.y, -thrustVector.z);


    // do not allow very steep desired bank angles
    desBankPitch = CONSTRAIN(desBankPitch, -.5f, .5f);
    desBankRoll = CONSTRAIN(desBankRoll, -.5f, .5f);

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
    diffThrust.y = desMoment.y / L / 2.f / sqrtf(2) ;
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

  // PARAMETERS
  float Kp_pos_xy;
  float Kp_pos_z;

  float Kp_vel_xy;
  float Kp_vel_z;

  float Kp_bank;
  float Kp_yaw;

  V3F Kp_angle_rate;
};
