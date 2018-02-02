#include "Common.h"
#include "BaseDynamics.h"
#include "Math/Random.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"

#ifdef _MSC_VER //  visual studio
#pragma warning(disable: 4267 4244 4996)
#endif

using namespace SLR;

BaseDynamics::BaseDynamics(string name)
{
  _name = name;
  _initialized = false;
  Initialize();
}

int BaseDynamics::Initialize()
{
  ParamsHandle config = SimpleConfig::GetInstance();

  _initialized = false;

  _vehicleType = -1; // see BaseDynamics.h for list of numbers

  // load in BaseDynamics-specific double-valued settings from the config in your inheritor
  xMin = config->Get("Sim.xMin", -10.f);
  yMin = config->Get("Sim.yMin", -10.f);;
  xMax = config->Get("Sim.xMax", 10.f);
  yMax = config->Get("Sim.yMax", 10.f);
  bottom = config->Get("Sim.bottom", 0.f);
  top = config->Get("Sim.top", 10.f);

  return 1;
}

void BaseDynamics::ResetState(V3F newPos, V3F newVel, Quaternion<float> newAtt, V3F newOmega)
{
  omega = newOmega;
  pos = newPos;
  vel = newVel;
  quat = newAtt;

}

void BaseDynamics::SyncToVicon(GlobalPose gp)
{
  vel = V3F();
  omega = V3F();
  pos = gp.pos;
  quat = gp.q;

  printf("\nSIMULATOR_RESET ncommand received;  set sim pose = vicon, and sim vel = 0.\n");
 }

GlobalPose BaseDynamics::GenerateGP(void)
{
  GlobalPose p;
  p.pos = pos;
  p.q = quat;
  return p;
}

bool BaseDynamics::GetData(const string& name, float& ret) const
{
  if (name.find_first_of(".") == string::npos) return false;
  string leftPart = LeftOf(name, '.');
  string rightPart = RightOf(name, '.');
  if (ToUpper(leftPart) == ToUpper(_name))
  {
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
    GETTER_HELPER("POS.X", pos.x);
    GETTER_HELPER("POS.Y", pos.y);
    GETTER_HELPER("POS.Z", pos.z);
    GETTER_HELPER("VEL.X", vel.x);
    GETTER_HELPER("VEL.Y", vel.y);
    GETTER_HELPER("VEL.Z", vel.z);
    GETTER_HELPER("YAW", quat.ToEulerYPR()[0]);
    GETTER_HELPER("PITCH", quat.ToEulerYPR()[1]);
    GETTER_HELPER("ROLL", quat.ToEulerYPR()[2]);
    GETTER_HELPER("OMEGA.X", omega.x);
    GETTER_HELPER("OMEGA.Y", omega.y);
    GETTER_HELPER("OMEGA.Z", omega.z);
#undef GETTER_HELPER
  }
  return false;
}

vector<string> BaseDynamics::GetFields() const
{
  vector<string> ret;
  ret.push_back(_name + ".Pos.X");
  ret.push_back(_name + ".Pos.Y");
  ret.push_back(_name + ".Pos.Z");
  ret.push_back(_name + ".Vel.X");
  ret.push_back(_name + ".Vel.Y");
  ret.push_back(_name + ".Vel.Z");
  ret.push_back(_name + ".Yaw");
  ret.push_back(_name + ".Pitch");
  ret.push_back(_name + ".Roll");
  ret.push_back(_name + ".Omega.X");
  ret.push_back(_name + ".Omega.Y");
  ret.push_back(_name + ".Omega.Z");
  return ret;
}
