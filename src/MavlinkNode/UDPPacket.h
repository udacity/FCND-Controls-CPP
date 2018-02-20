#ifndef UDP_PACKET_H_FEB_27_2008_SVL5
#define UDP_PACKET_H_FEB_27_2008_SVL5

#define MAX_UDP_PACKET_SIZE 65467 // UDP protocol max message size

struct UDPPacket {
	unsigned short port;    
	int source_addr;
  int dest_addr;

	unsigned int len;
	unsigned char* data;
};

#endif //UDP_PACKET_H_FEB_27_2008_SVL5

