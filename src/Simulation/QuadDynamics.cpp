#include "Common.h"
#include "QuadDynamics.h"
#include "Math/Random.h"
#include "Math/Quaternion.h"
#include "matrix/math.hpp"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "ControllerFactory.h"

#ifdef _MSC_VER //  visual studio
#pragma warning(disable: 4267 4244 4996)
#endif

#define ONBOARD_TS 0.002 // Controller update dt in [s]. TODO AS PARAM!

QuadDynamics::QuadDynamics(string name) 
 : BaseDynamics(name)
{
  Initialize();
}

void QuadDynamics::Reset()
{
  // Reinitialise the vehicle
  Initialize();
}

void QuadDynamics::ResetState(V3F pos, V3F vel, Quaternion<float> att, V3F omega)
{
  BaseDynamics::ResetState(pos,vel,att,omega);

  controllerUpdateInterval = (double)ONBOARD_TS;
  timeSinceLastControllerUpdate = controllerUpdateInterval; // Force the simulation to start with a controller update

	xyzDisturbance = rotDisturbance = V3D();
}

void QuadDynamics::SetPosVelAttOmega(V3F pos, V3F vel, Quaternion<float> att, V3F omega)
{ 
  BaseDynamics::ResetState(pos,vel,att,omega);
}

int QuadDynamics::Initialize()
{
  if(!BaseDynamics::Initialize()) return 0;

  _initialized = false;
	
  _vehicleType = VEHICLE_TYPE_QUAD;

  _lastTrajPointTime = 0;
  _trajLogStepTime = 0;

  ParamsHandle config = SimpleConfig::GetInstance();
	
  // PARAMETERS
  M = config->Get(_name+".Mass", .5f);
  L = config->Get(_name + ".L", 0.17f); // dist from center to thrust
  cx = config->Get(_name + ".cx", 0.f);
  cy = config->Get(_name + ".cy", 0.f);

  // Moments of inertia
  Ixx = config->Get(_name + ".Ixx", 0.001f);
  Iyy = config->Get(_name + ".Iyy", 0.001f);
  Izz = config->Get(_name + ".Izz", 0.002f);;

  // motor dynamics (up and down taus)
  tauaUp = config->Get(_name + ".tauaUp", 0.01f);
  tauaDown = config->Get(_name + ".tauaDown", 0.02f);

  kappa = config->Get(_name + ".kappa", 0.01f);

  gyroNoiseInt = config->Get("Sim.gyroNoiseInt", 0.f);
  rotDisturbanceInt = config->Get("Sim.rotDisturbanceInt", 0.f);
  xyzDisturbanceInt = config->Get("Sim.xyzDisturbanceInt", 0.f);
  rotDisturbanceBW = config->Get("Sim.rotDisturbanceBW", 0.f);
  xyzDisturbanceBW = config->Get("Sim.xyzDisturbanceBW", 0.f);

  minMotorThrust = config->Get(_name + "minMotorThrust", .1f);
  maxMotorThrust = config->Get(_name + ".maxMotorThrust", 4.5f);

  randomMotorForceMag = config->Get(_name + ".randomMotorForceMag", 0.f);

  _trajLogStepTime = config->Get(_name + ".trajectoryLogStepTime", 0.f);

  _flightMode = config->Get(_name+".SimMode", "Full3D");

  ResetState(V3F());

  V3F trajOffset = config->Get(_name + ".TrajectoryOffset", V3F());
  float trajTimeOffset = config->Get(_name + ".TrajectoryTimeOffset", 0);

  string controlConfig = config->Get(_name + ".ControlConfig", "ControlParams");
  controller = CreateController(config->Get(_name + ".ControlType", "QuadControl") , controlConfig);
  if (controller)
  {
    controller->SetTrajTimeOffset(trajTimeOffset);
    controller->SetTrajectoryOffset(trajOffset);
    if (config->Get(controlConfig + ".UseIdealEstimator", 0) == 1)
    {
      updateIdealStateCallback = MakeDelegate(controller.get(), &BaseController::OverrideEstimates);
    }
  }
  else
  {
    SLR_WARNING1("Failed to create controller for %s", _name.c_str());
  }

  _lastPosFollowErr = 0;

  V3F ypr = config->Get(_name + ".InitialYPR", V3F());
  ResetState(config->Get(_name + ".InitialPos", V3F(0, 0, 1)),
    config->Get(_name + ".InitialVel", V3F()),
    Quaternion<float>::FromEulerYPR(ypr.x, ypr.y, ypr.z),
    config->Get(_name + ".InitialOmega", V3F()));

  _initialized = true;

  // Initialise the trajectory log
  //string followedTrajFile = string("../config/") + config->Get("Sim.LoggedStateFile", "");
  _followed_traj.reset(new Trajectory());
  //followed_traj->SetLogFile(followedTrajFile);
  followedTrajectoryCallback = MakeDelegate(_followed_traj.get(), &Trajectory::AddTrajectoryPoint);

  return 1;
}


void QuadDynamics::Run(float dt, float simulationTime, int &idum, V3F externalForceInGlobalFrame, V3F externalMomentInBodyFrame)
{
	if (dt <= 0 || dt>0.05 || _isnan(dt))
	{
		printf("Something is wrong with dt: %lf", dt);
	}
  double remainingTimeToSimulate = dt;

  while(remainingTimeToSimulate > 0.000001) // Time intervals lower than that are just discarded (for speed of running)
  {
    if(timeSinceLastControllerUpdate >= controllerUpdateInterval)
    {
      // generate virtual gyro and accelerometer data
			V3F newRawGyro = V3F(omega+sqrtf(gyroNoiseInt/dt)*V3F(gasdev(idum),gasdev(idum),gasdev(idum)));
			const float c = expf(-dt/0.004f); // the real gyro filter has 250Hz bandwidth
			_rawGyro = (1.f-c)*newRawGyro + c*_rawGyro;

			// ... accelerometer (no noise currently)
      
      V3F bodyAcc = quat.Rotate_ItoB(V3F(acc));
			V3F rawAccel = V3F( bodyAcc);
			rawAccel.constrain(-6.f*9.81f,6.f*9.81f);


      // TODO: run callbacks to update sensors
			// push this into the HAL to the simulated onboard controller
			//_onboard.SetIMU_AG(rawAccel,_rawGyro);

      //_onboard.SetRangeSensor(pos.z);
      //V3F vel_body = quat.Rotate_ItoB(vel);
      //_onboard.SetOpticalFlow(vel.x,vel.y); // todo - optical flow also sees rotation, and there's a scale thing here..

			// This is the update of the onboard controller -- runs timeout logic, sensor filtering, estimation, 
			// controller, and produces a new set of motor commands
			//_onboard.RunEstimation();
			if (updateIdealStateCallback) 
			{
				updateIdealStateCallback(Position(), Velocity(), quat, Omega());
			}
			if (controller)
			{
        curCmd = controller->RunControl(controllerUpdateInterval, simulationTime);
        _lastPosFollowErr = controller->curTrajPoint.position.dist(Position());
			}

      if (simulationTime < 0.0000001){
        motorCmdsOld(0) = curCmd.desiredThrustsN[0];
        motorCmdsOld(1) = curCmd.desiredThrustsN[1];
        motorCmdsOld(2) = curCmd.desiredThrustsN[2];
        motorCmdsOld(3) = curCmd.desiredThrustsN[3];
      }

      timeSinceLastControllerUpdate = 0.0;
    }

    const double simStep = MIN(controllerUpdateInterval - timeSinceLastControllerUpdate, remainingTimeToSimulate);
    Dynamics(simStep, simulationTime, externalForceInGlobalFrame, externalMomentInBodyFrame, idum);
    timeSinceLastControllerUpdate += simStep;
    remainingTimeToSimulate -= simStep;
  }
}


void QuadDynamics::Dynamics(float dt, float simTime, V3F external_force, V3F external_moment, int& idum)
{
  // NED/FRD reference frame
  matrix::Vector<float,3> ext_moment;
  ext_moment(0) = external_moment.x;
  ext_moment(1) = external_moment.y;
  ext_moment(2) = external_moment.z;

  // constrain the desired thrusts to reflect real-world constraints
  for (int i = 0; i < 4; i++)
  {
		curCmd.desiredThrustsN[i] = CONSTRAIN(curCmd.desiredThrustsN[i], minMotorThrust, maxMotorThrust);
    motorCmdsN(i) = curCmd.desiredThrustsN[i] + randomMotorForceMag * ran1_inRange(-1.f, 1.f, idum);
  }

  // Prop dynamics, props cannot change thrusts in a non continuous manner
  for (int m = 0; m < 4; m++){
    if (motorCmdsN(m) >= motorCmdsOld(m))
    {
      motorCmdsN(m) = motorCmdsN(m) * ((float)dt/((float)dt + (float)tauaUp)) + motorCmdsOld(m) * ((float)tauaUp/((float)dt + (float)tauaUp));
    }
    else 
    {
      motorCmdsN(m) = motorCmdsN(m) * ((float)dt/((float)dt + (float)tauaDown)) + motorCmdsOld(m) * ((float)tauaDown/((float)dt + (float)tauaDown));
    }
  }

  float total_thrust = motorCmdsN(0) + motorCmdsN(1) + motorCmdsN(2) + motorCmdsN(3);
  V3F oldPos;
  V3F force_body_frame(0.f,0.f,-total_thrust);
  float half_dt = dt/2;
  V3F force_inertial_frame = quat.Rotate_BtoI(force_body_frame);
  V3F gravity(0.f,0.f,9.81f);


  matrix::Vector<float,3> omega_v;
  omega_v(0) = omega.x;
  omega_v(1) = omega.y;
  omega_v(2) = omega.z;

  matrix::Matrix<float,3,3> omega_skew;
  omega_skew(0,0) = 0;
  omega_skew(1,1) = 0;
  omega_skew(2,2) = 0;
  omega_skew(0,1) = -omega_v(2);
  omega_skew(0,2) = omega_v(1);
  omega_skew(1,0) = omega_v(2);
  omega_skew(1,2) = -omega_v(0);
  omega_skew(2,0) = -omega_v(1);
  omega_skew(2,1) = omega_v(0);

  // X shaped Quad Motor Matrix (NED Frame again)
  matrix::Matrix<float,3,4> motor_matrix;
  motor_matrix(0,0) = L/sqrt(2) + cy;
  motor_matrix(0,1) = -(L/sqrt(2) - cy);
  motor_matrix(0,2) = L/sqrt(2) + cy;
  motor_matrix(0,3) = -(L/sqrt(2) - cy);
  motor_matrix(1,0) = L/sqrt(2) - cx;
  motor_matrix(1,1) = L/sqrt(2) - cx;
  motor_matrix(1,2) = -(L/sqrt(2) + cx);
  motor_matrix(1,3) = -(L/sqrt(2) + cx);
  motor_matrix(2,0) = -kappa;
  motor_matrix(2,1) = kappa;
  motor_matrix(2,2) = kappa;
  motor_matrix(2,3) = -kappa;

  // Create Inertia matrix and inverse
  matrix::SquareMatrix<float,3> inertia_matrix;
  inertia_matrix.setZero();
  inertia_matrix(0,0) = (float)Ixx;
  inertia_matrix(1,1) = (float)Iyy;
  inertia_matrix(2,2) = (float)Izz;
  matrix::SquareMatrix<float,3> inv_inertia = matrix::inv(inertia_matrix);

  matrix::Vector<float,3> Iw;
  Iw = inertia_matrix*omega_v;
  matrix::Vector<float,3> total_moment = motor_matrix * motorCmdsN + ext_moment;

  // Individual Flight Mode kinematic equations

  if(_flightMode == "AttitudeOnly")
  {
    // Attitude only - no position changes
    acc = (force_inertial_frame + external_force)/M + gravity;
    vel = vel + acc * dt;
    quat = quat.IntegrateBodyRate_fast(omega.x, omega.y, omega.z, half_dt);
    omega_v = omega_v + inv_inertia * (total_moment - omega_skew*Iw) * dt;
  }
  if(_flightMode == "PlanarXZ")
  {
    // Planar XZ
    acc = (force_inertial_frame + external_force)/M + gravity;
    acc.y = 0; // no acceleration in y direction
    vel = vel + acc * dt;
    vel.y = 0; // no velocity in y direction
    oldPos = pos;
    pos = oldPos + vel * dt;
    pos.y = oldPos.y;
    quat = quat.IntegrateBodyRate_fast(omega.x, omega.y, omega.z, half_dt);
    omega_v = omega_v + inv_inertia * (total_moment - omega_skew*Iw) * dt;
    omega_v(0) = 0; // Roll is not allowed
    omega_v(2) = 0; // Yaw is also not allowed
  }
  if(_flightMode == "Full3D")
  {
    acc = (force_inertial_frame + external_force)/M + gravity;
    vel = vel + acc * dt;
    oldPos = pos;
    pos = oldPos + vel * dt;
    quat = quat.IntegrateBodyRate_fast(omega.x, omega.y, omega.z, half_dt);
    omega_v = omega_v + inv_inertia * (total_moment - omega_skew*Iw) * dt;
  }

  omega.x = omega_v(0);
  omega.y = omega_v(1);
  omega.z = omega_v(2);

  RunRoomConstraints(oldPos);

  motorCmdsOld = motorCmdsN;

  if (followedTrajectoryCallback)
  {
    if ((simTime - _lastTrajPointTime) > _trajLogStepTime)
    {
      _lastTrajPointTime = simTime;

      TrajectoryPoint traj_pt;
      traj_pt.time = _lastTrajPointTime;
      traj_pt.position = pos;
      traj_pt.velocity = vel;
      traj_pt.omega = omega;
      traj_pt.attitude = quat;

      followedTrajectoryCallback(traj_pt);
    }

    
  }
}

void QuadDynamics::RunRoomConstraints(const V3F& oldPos)
{
  /////////////////////////////////////////////////////////////////////
  // freeze out of bounds position states - this is what lets you
  // "(run up and down) the walls" instead of fly through them

  // "sticky floor" - if we're sitting on the floor, we shouldn't be drifting.
  if(pos[2]>=bottom)
  {
    pos = oldPos;
    pos[2] = bottom;
    vel[0] = vel[1] = 0;
    vel[2] = MIN(0,vel[2]);
    omega = V3F();
  }

  if((pos[0] < xMin) || (pos[0] > xMax))
  {
    pos[0] = oldPos[0];
    vel[0] = 0.0;
  }
  if((pos[1] < yMin) || (pos[1] > yMax))
  {
    pos[1] = oldPos[1];
    vel[1] = 0.0;
  }
  if((pos[2] > bottom) || (pos[2] < -top))
  {
    pos[2] = oldPos[2];
    vel[2] = 0.0;
  }
}

void QuadDynamics::SetCommands(const VehicleCommand& cmd)
{
	curCmd = cmd;
}


void QuadDynamics::TurnOffNonidealities()
{
	// This function turns off all nonidealities in the simulator.

	//Set noise terms to zero.
	gyroNoiseInt = 0; //gyroNoiseInt
	rotDisturbanceInt = 0; //rotDistInt
	xyzDisturbanceInt = 0; //xyzDistInt
	rotDisturbanceBW = 0; //rotDistBW
	xyzDisturbanceBW = 0; //xyzDistBW

	//Meters error in CMToSpine
	//Reload cx and cy because there has been added an error to them,
	//see initialized in QuadDynamics::Initialize
    cx = 0;
    cy = 0;
}

bool QuadDynamics::GetData(const string& name, float& ret) const
{
  if (name.find_first_of(".") == string::npos) return false;
  string leftPart = LeftOf(name, '.');
  string rightPart = RightOf(name, '.');
  if (ToUpper(leftPart) == ToUpper(_name))
  {
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
    // UDACITY CONVENTION
    GETTER_HELPER("Thrust.A", motorCmdsN(0));
    GETTER_HELPER("Thrust.B", motorCmdsN(1));
    GETTER_HELPER("Thrust.C", motorCmdsN(2));
    GETTER_HELPER("Thrust.D", motorCmdsN(3));
    GETTER_HELPER("PosFollowErr", _lastPosFollowErr);
#undef GETTER_HELPER
    return BaseDynamics::GetData(name, ret);
  }

  if (controller)
  {
    return controller->GetData(name, ret);
  }

  return false;  
}

vector<string> QuadDynamics::GetFields() const
{
  vector<string> ret = BaseDynamics::GetFields();
  ret.push_back(_name + ".Thrust.A");
  ret.push_back(_name + ".Thrust.B");
  ret.push_back(_name + ".Thrust.C");
  ret.push_back(_name + ".Thrust.D");
  ret.push_back(_name + ".PosFollowErr");
  
  if (controller)
  {
    vector<string> controllerFields = controller->GetFields();
    ret.insert(ret.end(), controllerFields.begin(), controllerFields.end());
  }
  return ret;
}
