#include "Common.h"
#include "Math/Quaternion.h"
#include <vector>
using namespace std;

#ifdef __APPLE__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Waddress-of-packed-member"
#endif
#include "mavlink/common/mavlink.h"
#ifdef __APPLE__
#pragma clang diagnostic pop
#endif

vector<uint8_t> MakeMavlinkPacket_LocalPose(float simTime, V3F pos, V3F vel)
{
  vector<uint8_t> ret;
  ret.resize(MAVLINK_MAX_PACKET_LEN);
  mavlink_message_t msg;

  mavlink_msg_local_position_ned_pack(1, 200, &msg, (int)(simTime*1e6f),
    pos[0], pos[1], pos[2],
    vel[0], vel[1], vel[2]);

  int len = mavlink_msg_to_send_buffer(&ret[0], &msg);
  ret.resize(len);
  return ret;
}

vector<uint8_t> MakeMavlinkPacket_Heartbeat()
{
  vector<uint8_t> ret;
  ret.resize(MAVLINK_MAX_PACKET_LEN);
  mavlink_message_t msg;

  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);

  int len = mavlink_msg_to_send_buffer(&ret[0], &msg);
  ret.resize(len);
  return ret;
}

vector<uint8_t> MakeMavlinkPacket_Status()
{
  vector<uint8_t> ret;
  ret.resize(MAVLINK_MAX_PACKET_LEN);
  mavlink_message_t msg;

  /* Send Status */
  mavlink_msg_sys_status_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);

  int len = mavlink_msg_to_send_buffer(&ret[0], &msg);
  ret.resize(len);
  return ret;
}

vector<uint8_t> MakeMavlinkPacket_Attitude(float simTime, SLR::Quaternion<float> attitude, V3F omega)
{
  vector<uint8_t> ret;
  ret.resize(MAVLINK_MAX_PACKET_LEN);
  mavlink_message_t msg;

  mavlink_msg_attitude_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, (int)(simTime*1e6f), attitude.Roll(), attitude.Pitch(), attitude.Yaw(), omega.x, omega.y, omega.z);

  int len = mavlink_msg_to_send_buffer(&ret[0], &msg);
  ret.resize(len);
  return ret;
}

