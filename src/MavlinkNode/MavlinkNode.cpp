#include "Common.h"
#include "MavlinkNode.h"
#ifndef _WIN32
#include <pthread.h>
#endif

MavlinkNode::MavlinkNode(string myIP)
	: _socket(myIP, MAVLINK_RX_PORT)
{
	_first = true;
	_doubleCnt=0;

	_running = true;

#ifdef _WIN32
	_thread = CreateThread(NULL,NULL,RxThread,this,NULL,NULL);
#else
	pthread_create(&_thread, NULL, RxThread, this);
#endif

	_packet.data = new unsigned char[MAX_UDP_PACKET_SIZE];
}

MavlinkNode::~MavlinkNode()
{
	_running = false;
#ifdef _WIN32
	if(WaitForSingleObject(_thread,100)!=WAIT_OBJECT_0)
	{
		TerminateThread(_thread,10);
	}
#else
	_socket.shutdown();
#ifdef __APPLE__
	pthread_cancel(_thread);
#endif
	pthread_join(_thread, NULL);
#endif
	delete [] _packet.data;
}

#ifdef _WIN32
DWORD WINAPI MavlinkNode::RxThread(LPVOID param)
#else
void* MavlinkNode::RxThread(void* param)
#endif
{
	int numRead;
	string srcAddr;
	unsigned short srcPort;

	MavlinkNode* p = (MavlinkNode*)param;
  while (p->_running)
  {
    try
    {
      numRead = p->_socket.recvFrom(p->_packet.data, MAX_UDP_PACKET_SIZE, srcAddr, srcPort);
    }
    catch (...)
    {
      continue;
    }
		
		if(numRead<0)
		{
			// error
		}
		else
		{
			p->_packet.len = numRead;
			p->UDPPacketCallback(p->_packet);
		}
	}
	
	return 0;
}

void MavlinkNode::Send(const vector<uint8_t>& packet)
{
  _socket.sendTo(&packet[0], (int)packet.size(), "127.0.0.1", MAVLINK_TX_PORT);
}

void MavlinkNode::UDPPacketCallback(UDPPacket& m)
{
  // TODO

  mavlink_message_t msg;
  mavlink_status_t status;

  //printf("Bytes Received: %d\n", (int)m.len);
  for (unsigned int i = 0; i < m.len; ++i)
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, m.data[i], &msg, &status))
    {
      // Packet received
      //printf("Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
    }
  }

	/*if(!callback.empty()){
		callback(ret,callbackArg);
	}*/
}
