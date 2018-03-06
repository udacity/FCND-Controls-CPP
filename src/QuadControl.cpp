#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Simulation/BaseDynamics.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"


void QuadControl::Init()
{
  BaseController::Init();
    
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  Kp_pos_xy = config->Get(_config+".Kp_pos_xy", 0);
  Kp_pos_z = config->Get(_config + ".Kp_pos_z", 0);
     
  Kp_vel_xy = config->Get(_config + ".Kp_vel_xy", 0);
  Kp_vel_z = config->Get(_config + ".Kp_vel_z", 0);

  Kp_bank = config->Get(_config + ".Kp_bank", 0);
  Kp_yaw = config->Get(_config + ".Kp_yaw", 0);

  PGain_Omega = config->Get(_config + ".Kp_pqr", V3F());

  max_descent_rate = config->Get(_config + ".maxDescentRate", 100);
  max_ascent_rate = config->Get(_config + ".maxAscentRate", 100);
  max_speed_xy = config->Get(_config + ".maxSpeedXY", 100);
  max_horiz_accel = config->Get(_config + ".maxHorizAccel", 100);

  max_tilt_angle = config->Get(_config + ".maxTiltAngle", 100);

  min_motor_thrust = config->Get(_config + ".minMotorThrust", 0);
  max_motor_thrust = config->Get(_config + ".maxMotorThrust", 100);
}

VehicleCommand QuadControl::GenerateMotorCommands(float desCollectiveThrust, V3F desMoment)
{
  // Convert a desired 3-axis moment and collective thrust command to 
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

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F body_rate_cmd, V3F body_rate)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   body_rate_cmd: desired body rates [rad/s]
  //   body_rate: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter Kp_angle_rate (it's a V3F)

  V3F moment_cmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return moment_cmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::ReducedAttitudeControl(V3F desAccel, Quaternion<float> attitude, float desCollectiveThrust)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   desAccel: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   desCollectiveThrust: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F can be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain Kp_bank
  //  - desCollectiveThrust is a force! You'll likely want to convert it to acceleration first

  V3F desOmega;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////



  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return desOmega;
}

float QuadControl::AltitudeControl(float desPosZ, float desVelZ, float posZ, float velZ, Quaternion<float> attitude, float accelFF)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   desPosZ, desVelZ: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelFF: feed-forward vertical acceleration in NED [m/s2]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters Kp_pos_z and Kp_vel_z
  //  - max_ascent_rate and max_descent_rate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////



  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::HorizontalControl(V3F position_cmd, V3F velocity_ff, V3F position, V3F velocity, V3F acceleration_ff)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   position_cmd: desired position, in NED [m]
  //   velocity_ff: desired velocity, in NED [m/s]
  //   position: current position, NED [m]
  //   velocity: current velocity, NED [m/s]
  //   acceleration_ff: desired acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use fmodf(foo,b) to constrain float foo to range [0,b]
  //  - use the gain parameters Kp_pos_xy and Kp_vel_xy
  //  - make sure you cap the horizontal velocity and acceleration
  //    to max_speed_xy and max_horiz_accel

  // make sure we don't have any incoming z-component
  acceleration_ff.z = 0;
  velocity_ff.z = 0;
  position_cmd.z = position.z;

  V3F accel_cmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accel_cmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yaw_cmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yaw_cmd
  // INPUTS: 
  //   yaw_cmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to constrain float foo to range [0,b]
  //  - use the yaw control gain parameter Kp_yaw

  float yaw_rate_cmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yaw_rate_cmd;

}

VehicleCommand QuadControl::RunControl(float dt, float sim_time)
{
  curTrajPoint = GetNextTrajectoryPoint(sim_time);

  float desCollThrust = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(max_motor_thrust - min_motor_thrust);
  desCollThrust = CONSTRAIN(desCollThrust, (min_motor_thrust+ thrustMargin)*4.f, (max_motor_thrust-thrustMargin)*4.f);
  
  V3F desAcc = HorizontalControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = ReducedAttitudeControl(desAcc, estAtt, desCollThrust);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(desCollThrust, desMoment);
}
