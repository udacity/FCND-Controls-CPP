#pragma once

vector<uint8_t> MakeMavlinkPacket_LocalPose(float simTime, V3F pos, V3F vel);
vector<uint8_t> MakeMavlinkPacket_Heartbeat();
vector<uint8_t> MakeMavlinkPacket_Status();
vector<uint8_t> MakeMavlinkPacket_Attitude(float simTime, Quaternion<float> attitude, V3F omega);