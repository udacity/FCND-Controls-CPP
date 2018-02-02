#pragma once

#include "Math/Quaternion.h"
#include "Utility/FixedQueue.h"
#include "VehicleDatatypes.h"
#include <vector>

using namespace SLR;

#define MAX_TRAJECTORY_POINTS 10000

class Trajectory {
public:
  Trajectory();
  Trajectory(const string& filename);
  ~Trajectory();
  bool ReadFile(const string& filename);
  void ParseLine(const string& filename, const string& s);
  void Clear();
  void SetLogFile(const string& filename);
  void AddTrajectoryPoint(TrajectoryPoint traj_pt);
  TrajectoryPoint NextTrajectoryPoint(float time);
  void WriteTrajectoryPointToFile(FILE* f, TrajectoryPoint traj_pt);

  FixedQueue<TrajectoryPoint> traj; // vector containing the trajectory points

  int GetCurTrajectoryPoint() const { return _curTrajPoint; }
private:
  string _log_filename;
  FILE* _log_file;
  int _curTrajPoint;
};
