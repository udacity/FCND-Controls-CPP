#pragma once

#include <vector>
#include "DataSource.h"
#include "Utility/FastDelegate.h"
#include "VehicleDatatypes.h"
#include "Trajectory.h"
using namespace fastdelegate;
using namespace SLR;
using namespace std;

class BaseController;
typedef shared_ptr<BaseController> ControllerHandle;

class BaseController : public DataSource
{
public:
  BaseController(string config);
  virtual ~BaseController() {};

  virtual void RunEstimation() {};
  virtual VehicleCommand RunControl(float dt, float sim_time) { return VehicleCommand(); };

  virtual void Init();
  virtual void Reset();

  void OnSensor_IMU(V3F accel, V3F gyros);
  void OnSensor_OpticalFlow(float x, float y);
  void OnSensor_Range(float z);
  void OnSensor_GPS(V3D LLA); 
  void OnSensor_Magnetometer(V3F);

  TrajectoryPoint GetNextTrajectoryPoint(float sim_time);

  // Allows the simulator to provide perfect state data to the controller
  void OverrideEstimates(V3F pos, V3F vel, Quaternion<float> attitude, V3F omega);

  // Access functions for graphing variables
  virtual bool GetData(const string& name, float& ret) const;
  virtual vector<string> GetFields() const;

  // system parameters params
  float M; // mass
  float L; // length of arm from centre of quadrocopter to motor
  float Ixx, Iyy, Izz; // mass moment of inertia / second moment of inertia
  float kappa; // torque (Nm) produced by motor per N of thrust produced

  // controller input (reference state)
  int mode;
  V3F desiredPos;
  V3F desOmegaDot;
  float desYaw;
  VehicleCommand cmd;

  // controller intermediates
  V3F desOmega;
  float desCollThrust;
  
  // Estimator state
  Quaternion<float> estAtt;
  V3F estVel;
  V3F estPos;
  V3F estOmega;

  // measurements
  float optFlowX, optFlowY;
  V3F gyros;
  V3F accels;
  float range;

  Trajectory trajectory;
  string _config;
};