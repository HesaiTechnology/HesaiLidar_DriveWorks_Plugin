
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>

#include "InputSocket.h"
#include "platUtil.h"

static const size_t packet_size = sizeof(UdpPacket().m_u8Buf);

void InputSocket::init(std::string deviceipaddr, std::string hostIpAddr, std::string multcastIpAddr, uint16_t lidarport, uint16_t gpsport) {
	m_sDeviceIpAddr = deviceipaddr;
	m_sMultcastIpAddr = multcastIpAddr;
	m_u16LidarPort = lidarport;
	m_sUdpVresion = "";
	m_bGetUdpVersion = false;
	m_iTimestampIndex = 0;
	m_iUtcIindex = 0;
	m_iSequenceNumberIndex = 0;
	m_iPacketSize = 0;
	// if(!m_sDeviceIpAddr.empty())
	// 	printf("Accepting packets from IP address: %s\n", m_sDeviceIpAddr.c_str());
	m_iSockfd = -1;
	m_iSockGpsfd = -1;
	m_u32Sequencenum = 0;

	// printf("InputSocket: init, UDP port=%d, multcastIp=%s, gpsport=%d\n", lidarport, multcastIpAddr.c_str(), gpsport);
	m_iSockfd = socket(PF_INET, SOCK_DGRAM, 0);
	if(m_iSockfd == -1) {
		perror("socket");
		return;
  	}

	sockaddr_in my_addr;
	memset(&my_addr, 0, sizeof(my_addr));
	// host byte order
	my_addr.sin_family = AF_INET;
	// port in network byte order, 2368
	my_addr.sin_port = htons(lidarport);
	// automatically fill in my IP
	my_addr.sin_addr.s_addr = INADDR_ANY;

	if(bind(m_iSockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
		perror("bind error");
		return;
	}
	int nRecvBuf = 400000000;
	setsockopt(m_iSockfd, SOL_SOCKET, SO_RCVBUF, (const char*)&nRecvBuf, sizeof(int));
	  int curRcvBufSize = -1;
     socklen_t optlen = sizeof(curRcvBufSize);
     if (getsockopt(m_iSockfd, SOL_SOCKET, SO_RCVBUF, &curRcvBufSize, &optlen) < 0)
     {
         printf("getsockopt error=%d(%s)!!!\n", errno, strerror(errno));
     }
    //  printf("OS current udp socket recv buff size is: %d\n", curRcvBufSize);

	if(fcntl(m_iSockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
		perror("non-block");
		return;
	}
	m_iSocktNumber = 1;

	if(0 != gpsport) {
		m_iSockGpsfd = socket(PF_INET, SOCK_DGRAM, 0);
		if (m_iSockGpsfd == -1) {
			perror("socket");  // TODO: perror errno.
			return;
		}

		sockaddr_in myAddressGPS;                        // my address information
		memset(&myAddressGPS, 0, sizeof(myAddressGPS));  // initialize to zeros
		myAddressGPS.sin_family = AF_INET;               // host byte order
		myAddressGPS.sin_port = htons(gpsport);          // port in network byte order
		myAddressGPS.sin_addr.s_addr = INADDR_ANY;  	 // automatically fill in my IP

		if (bind(m_iSockGpsfd, reinterpret_cast<sockaddr *>(&myAddressGPS), sizeof(sockaddr)) == -1) {
			perror("bind");  // TODO: perror errno
			return;
		}

		if (fcntl(m_iSockGpsfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
			perror("non-block");
			return;
		}
		m_iSocktNumber = 2;
	}
	// int get_error = getsockopt(m_iSockfd, SOL_SOCKET, SO_NO_CHECK, &getchecksum, &option_int);
	int nochecksum = 1;
	setsockopt(m_iSockfd, SOL_SOCKET, SO_NO_CHECK, &nochecksum, sizeof(nochecksum));
	// printf("Udp socket initialized, fd = %d\n", m_iSockfd);
	if(m_sMultcastIpAddr != ""){
		struct ip_mreq mreq;                    
		mreq.imr_multiaddr.s_addr=inet_addr(m_sMultcastIpAddr.c_str());
		// configurate the socket with hostip, then broadcast is available
		mreq.imr_interface.s_addr = hostIpAddr == "" ? htonl(INADDR_ANY) : inet_addr(hostIpAddr.c_str());
		int ret = setsockopt(m_iSockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq));
		if (ret < 0) {
			perror("Multicast IP error,set correct multicast ip address or keep it empty\n");
		} 
		else {
			printf("Recive data from multicast ip address %s\n", m_sMultcastIpAddr.c_str());
		}
  	}
}

void InputSocket::close_socket() { 
	if(m_iSockGpsfd >0) close(m_iSockGpsfd);
	if(m_iSockfd >0) close(m_iSockfd); 
}

PacketType InputSocket::getPacket(UdpPacket *&pkt, int timeout) {
	// printf("InputSocket: getPacket, starting\n");
	timespec time;
	memset(&time, 0, sizeof(time));

	struct pollfd fds[m_iSocktNumber];
	if(m_iSocktNumber == 2) {
		fds[0].fd = m_iSockGpsfd;
		fds[0].events = POLLIN;

		fds[1].fd = m_iSockfd;
		fds[1].events = POLLIN;
	} 
	else if(m_iSocktNumber == 1) {
		fds[0].fd = m_iSockfd;
		fds[0].events = POLLIN;
	}
	sockaddr_in sender_address;
	socklen_t sender_address_len = sizeof(sender_address);
	int retval = poll(fds, m_iSocktNumber, timeout);
	if(retval < 0) { // poll() error?
		if(errno != EINTR) printf("poll() error: %s\n", strerror(errno));
		return ERROR_PACKET;
	}
	if(retval == 0) { // poll() timeout?
		printf("InputSocket: getPacket, Pandar poll() timeout\n");
		return TIMEOUT;
	}
	if((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||(fds[0].revents & POLLNVAL)) { // device error?
		printf("InputSocket: getPacket, poll() reports Pandar error\n");
		return ERROR_PACKET;
	}

	ssize_t nbytes = 0;
  	for (int i = 0; i != m_iSocktNumber; ++i) {
    	if (fds[i].revents & POLLIN) {
      		nbytes = recvfrom(fds[i].fd, &pkt->m_u8Buf[0], 10000, 0, (sockaddr *)&sender_address, &sender_address_len);
			// pkt->m_i16Len = nbytes;
			// printf("fds[%d] size: %d\n",i, nbytes);
      		break;
    	}
  	}
	if (nbytes == 512) {
		// ROS_ERROR("GPS");
		return GPS_PACKET;
	}
	if (nbytes == FAULT_MESSAGE_PCAKET_SIZE) {
		return FAULT_MESSAGE_PACKET;
	}
	if (nbytes == LOG_REPORT_PCAKET_SIZE) {
		return LOG_REPORT_PACKET;
	}

	return POINTCLOUD_PACKET;
}