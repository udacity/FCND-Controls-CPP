#pragma once

#include "PracticalSocket.h"
#include "Utility/FastDelegate.h"
#include "UDPPacket.h"

#ifdef __APPLE__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Waddress-of-packed-member"
#endif
#include "mavlink/common/mavlink.h"
#ifdef __APPLE__
#pragma clang diagnostic pop
#endif

#include <vector>
using namespace fastdelegate;
using std::vector;

#define MAVLINK_TX_PORT 14555
#define MAVLINK_RX_PORT 14550 

typedef FastDelegate2<mavlink_message_t, const UDPPacket&> MavlinkNodeCallback;

class MavlinkNode
{
public:
	MavlinkNode(string myIP="127.0.0.1");
	~MavlinkNode();

#ifdef _WIN32
	static DWORD WINAPI RxThread(LPVOID param);
#else
	static void* RxThread(void* param);
#endif

	void SetCallback(MavlinkNodeCallback callback, void* arg)
  {
		this->callback = callback;
		this->callbackArg = arg;
	}	

	void ClearCallback()
	{
		this->callback.clear();
	}

  void Send(const vector<uint8_t>& packet);

private:
	void UDPPacketCallback(UDPPacket& m);

  MavlinkNodeCallback callback;
	void* callbackArg;

	unsigned short _lastSeqNum;
	bool _first;
	unsigned int _doubleCnt;

	UDPSocket _socket;
#ifdef _WIN32
	HANDLE _thread;
#else
	pthread_t _thread;
#endif
	UDPPacket _packet;
	bool _running;
};
