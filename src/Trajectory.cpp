#include "Common.h"
#include "Trajectory.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"

using namespace SLR;

Trajectory::Trajectory() : traj(MAX_TRAJECTORY_POINTS) 
{
  _log_file = NULL;
}

Trajectory::Trajectory(const string& filename) : traj(MAX_TRAJECTORY_POINTS)
{
  ReadFile(filename);
}

Trajectory::~Trajectory()
{
  // Close any open logging files
  if (_log_file)
  {
    fclose(_log_file);
  }
}

bool Trajectory::ReadFile(const string& filename)
{
  traj.reset();

  FILE* f = fopen(filename.c_str(), "r");
  if (!f)
  {
    return false;
  }

  char buf[512];
  buf[511] = 0; // null char

  // read line by line...
  while (fgets(buf, 510, f))
  {
    string s(buf);

    ParseLine(filename, s);
  }

  fclose(f);

  // Handle empty trajectory files
  // check the length of the trajectory vector
  // if there are no points in the trajectory file, then use the initial position as the only trajectory point
  if (traj.n_meas() == 0)
  {
    ParamsHandle config = SimpleConfig::GetInstance();
    TrajectoryPoint traj_pt;
    // TODO: no quad naming here.
    traj_pt.position = config->Get("Quad.InitialPos", V3F());
    traj_pt.velocity = config->Get("Quad.InitialVel", V3F());
    traj_pt.omega = config->Get("Quad.InitialOmega", V3F());
    V3F ypr = config->Get("Quad.InitialYPR", V3F());
    traj_pt.attitude = Quaternion<float>::FromEulerYPR(ypr[0], ypr[1], ypr[2]);

    traj.push(traj_pt);
  }

  return true;
}

void Trajectory::ParseLine(const string& filename, const string& s)
{
  std::size_t firstNonWS = s.find_first_not_of("\n\t ");

  // Ignore comments
  if (firstNonWS == std::string::npos || s[firstNonWS] == '#' || firstNonWS == '/')
  {
    return;
  }

  TrajectoryPoint traj_pt;

  V3F ypr; // Helper variable to read in yaw, pitch and roll
  sscanf(s.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &traj_pt.time, &traj_pt.position.x, &traj_pt.position.y, &traj_pt.position.z, &traj_pt.velocity.x, &traj_pt.velocity.y, &traj_pt.velocity.z, &ypr[0], &ypr[1], &ypr[2], &traj_pt.omega.x, &traj_pt.omega.y, &traj_pt.omega.z);

  // Convert yaw, pitch, and roll to an attitude quaternion
  traj_pt.attitude = Quaternion<float>::FromEulerYPR(ypr[0], ypr[1], ypr[2]);

  // Add the trajectory point to the vector of all trajectory points
  traj.push(traj_pt);
}

void Trajectory::Clear()
{
  _curTrajPoint = 0;
  traj.reset();

  // close and reopen the log file
  if (_log_file)
  {
    fclose(_log_file);
    _log_file = nullptr;
  }

  if (!_log_filename.empty())
  {
    _log_file = fopen(_log_filename.c_str(), "w");
  }
}

void Trajectory::SetLogFile(const string& filename)
{
  _log_filename = filename;

  // Close any file that might have been open and open the new file
  if (_log_file)
  {
    fclose(_log_file);

    if (_log_filename != "")
    {
      _log_file = fopen(_log_filename.c_str(), "w");
    }
  }
}

void Trajectory::AddTrajectoryPoint(TrajectoryPoint traj_pt)
{
  traj.push(traj_pt);

  // If there is a log file, write the point to file
  if (_log_file)
  {
    WriteTrajectoryPointToFile(_log_file, traj_pt);
  }
}

TrajectoryPoint Trajectory::NextTrajectoryPoint(float time)
{
  if (traj.empty()) return TrajectoryPoint();

  // Loop through the trajectory vector and get the next trajectory point
  for (int i = traj.n_meas()-1; i >= 0; i--)
  {
    if(traj.at(i).time < time)
    {
      _curTrajPoint = i;
      return traj.at(i);
    }
  }

  _curTrajPoint = traj.n_meas() - 1;
  // I should not get here
  return traj.newest();
}

void Trajectory::WriteTrajectoryPointToFile(FILE* f, TrajectoryPoint traj_pt)
{
  if (!f)
  {
    return;
  }

  // Write the trajectory point to file
  V3D ypr = traj_pt.attitude.ToEulerYPR();
  fprintf (f, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", (double)traj_pt.time, (double)traj_pt.position.x, (double)traj_pt.position.y, (double)traj_pt.position.z, (double)traj_pt.velocity.x, (double)traj_pt.velocity.y, (double)traj_pt.velocity.z, ypr[0], ypr[1], ypr[2], (double)traj_pt.omega.x, (double)traj_pt.omega.y, (double)traj_pt.omega.z);

  // Flush to file
  fflush(f);
}
