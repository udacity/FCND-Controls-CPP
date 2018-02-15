#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Simulation/BaseDynamics.h"
#include "Trajectory.h"
#include "BaseController.h"


void QuadControl::Init()
{
  BaseController::Init();
    
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  Kp_pos_xy = config->Get(_config+".PGain_Pos_XY", 0);
  Kp_pos_z = config->Get(_config + ".PGain_Pos_Z", 0);
     
  Kp_vel_xy = config->Get(_config + ".PGain_Vel_XY", 0);
  Kp_vel_z = config->Get(_config + ".PGain_Vel_Z", 0);

  Kp_bank = config->Get(_config + ".PGain_Bank", 0);
  Kp_yaw = config->Get(_config + ".PGain_Yaw", 0);

  Kp_angle_rate = config->Get(_config + ".PGain_Omega", V3F());
}

VehicleCommand QuadControl::GenerateMotorCommands(float desCollectiveThrust, V3F desMoment)
{
  //   Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   desCollectiveThrust: desired collective thrust [N]
  //   desMoment: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of desMoment via e.g. desMoment.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  cmd.desiredThrustsN[0] = M * 9.81f / 4.f; // front left
  cmd.desiredThrustsN[1] = M * 9.81f / 4.f; // front right
  cmd.desiredThrustsN[2] = M * 9.81f / 4.f; // rear left
  cmd.desiredThrustsN[3] = M * 9.81f / 4.f; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  // Convert desired moment into differential thrusts
  V3F diffThrust;

  // for X shaped quad
  diffThrust.x = desMoment.x / L / 2.f / sqrtf(2);
  diffThrust.y = desMoment.y / L / 2.f / sqrtf(2);
  diffThrust.z = desMoment.z / 4.f / kappa;

  // MIXING
  // combine the collective thrust with the differential thrust commands to find desired motor thrusts
  // X Shaped Quad (NED Frame)
  cmd.desiredThrustsN[0] = desCollectiveThrust / 4.f - diffThrust.z + diffThrust.y + diffThrust.x; // front left
  cmd.desiredThrustsN[1] = desCollectiveThrust / 4.f + diffThrust.z + diffThrust.y - diffThrust.x; // front right
  cmd.desiredThrustsN[2] = desCollectiveThrust / 4.f + diffThrust.z - diffThrust.y + diffThrust.x; // rear left
  cmd.desiredThrustsN[3] = desCollectiveThrust / 4.f - diffThrust.z - diffThrust.y - diffThrust.x; // rear right
  
  //////////////////////////////// END SOLUTION ///////////////////////////////

  return cmd;
}

// returns desired moments
V3F QuadControl::BodyRateControl(V3F body_rate_cmd, V3F body_rate)
{
  V3F rate_error = body_rate_cmd - body_rate;
  V3F omega_dot_des = rate_error * Kp_angle_rate;
  V3F moment_cmd = omega_dot_des * V3F(Ixx, Iyy, Izz);;
  return moment_cmd;
}

// returns a desired acceleration in global frame
V3F QuadControl::PositionControl(V3F position_cmd, V3F velocity_ff, V3F position, V3F velocity, V3F acceleration_ff)
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


// returns desired yaw rate
float QuadControl::YawControl(float yaw_cmd, float yaw)
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



// returns a desired roll and pitch rate 
V3F QuadControl::LateralControl(V3F acceleration_cmd, Quaternion<float> attitude)
{
  // get our current Euler angles
  float curRoll, curPitch, curYaw;
  attitude.ToEulerYPR(curRoll, curPitch, curYaw);

  // Simplistic thrust vector + desired bank angle calc
  V3F thrustVector = M * acceleration_cmd + V3F(0, 0, -M * 9.81f);

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

VehicleCommand QuadControl::AttitudeControl(V3F des_acceleration, float yaw_cmd)
{
  // Simplistic thrust vector + desired bank angle calc
  V3F thrustVector = M * des_acceleration + V3F(0, 0, -M * 9.81f);
  float desCollThrust = thrustVector.mag();

  V3F desOmega = LateralControl(des_acceleration, estAtt);
  desOmega.z = YawControl(yaw_cmd, estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(desCollThrust, desMoment);
}

VehicleCommand QuadControl::RunControl(float dt, float sim_time)
{
  curTrajPoint = GetNextTrajectoryPoint(sim_time);

  V3F desAcc = PositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

  return AttitudeControl(desAcc, curTrajPoint.attitude.Yaw());
}