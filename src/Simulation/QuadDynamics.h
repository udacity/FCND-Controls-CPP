#pragma once

#include "BaseDynamics.h"
#include "Math/V4D.h"
#include "BaseController.h"
#include <matrix/math.hpp>
#include "Math/LowPassFilter.h"

class QuadDynamics;
typedef shared_ptr<QuadDynamics> QuadcopterHandle;

class QuadDynamics : public BaseDynamics
{
public:
	static QuadcopterHandle Create(string name)
	{
		QuadcopterHandle ret(new QuadDynamics(name));
		return ret;
	}

  QuadDynamics(string name="");
	virtual ~QuadDynamics() {}; // destructor
	virtual int Initialize();

  virtual void Run(double dt, double simulationTime, int &idum,  // updates the simulation
      V3F externalForceInGlobalFrame = V3F(),    // required to take net forces into account
      V3F externalMomentInBodyFrame = V3F(),   // required to take net moments into account
                  string flightMode = "Full3D");
	virtual void SetCommands(const VehicleCommand& cmd);	// update commands in the simulator coming from a command2 packet

  virtual void Dynamics(double dt, V3F external_force, V3F external_moment, string flight_mode);

	double GetRotDistInt() {return rotDisturbanceInt;};
	double GetXyzDistInt() {return xyzDisturbanceInt;};
	double GetRotDistBW() {return rotDisturbanceBW;};
	double GetXyzDistBW() {return xyzDisturbanceBW;};
	double GetGyroNoiseInt() {return gyroNoiseInt;};

  virtual bool GetData(const string& name, float& ret) const;
  virtual vector<string> GetFields() const;

  void Reset();

	VehicleCommand GetCommands() const { return curCmd; }

  void ResetState(V3F pos=V3F(), V3F vel=V3F(), Quaternion<float> att=Quaternion<float>(), V3F omega=V3F());
  void SetPosVelAttOmega(V3F pos=V3F(), V3F vel=V3F(), Quaternion<float> att=Quaternion<float>(), V3F omega=V3F()); 

	void TurnOffNonidealities();

	void RunRoomConstraints(const V3F& oldPos);
  
	VehicleCommand curCmd;

	FastDelegate4<V3F, V3F, Quaternion<float>, V3F> updateIdealStateCallback;


  FastDelegate1<TrajectoryPoint> followedTrajectoryCallback;

	float GetArmLength() const { return L; }

  ControllerHandle controller;

protected:
  matrix::Vector<float, 4> motorCmdsN;
  matrix::Vector<float, 4> motorCmdsOld;


	// useful matrices/vectors that are recomputed from the state at each timestep
	double YPR[3];

  //////////////////////////////////////////////////////////////////
  // vehicle geometry and mass properties
  double cx;
  double cy;
  float L; // distance from body z axis to the prop

  // properties of the prop/motor system as modeled
  double tauaUp, tauaDown; // time constant
  float kappa; // Nm drag per N lift
  float minMotorThrust, maxMotorThrust;

	double xyzDisturbanceInt, xyzDisturbanceBW, rotDisturbanceInt, rotDisturbanceBW, gyroNoiseInt;
	V3D xyzDisturbance, rotDisturbance;

  double controllerUpdateInterval, timeSinceLastControllerUpdate;

	V3F _rawGyro;
};
