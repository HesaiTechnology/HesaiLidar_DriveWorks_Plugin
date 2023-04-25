#ifndef __PANDAR_INPUT_H
#define __PANDAR_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include <map>
#include "util.h"

#define FAULT_MESSAGE_PCAKET_SIZE (99)
#define LOG_REPORT_PCAKET_SIZE (273)

enum PacketType{
	POINTCLOUD_PACKET,
	ERROR_PACKET,
	GPS_PACKET,
	PCAP_END_PACKET,
	FAULT_MESSAGE_PACKET,
	LOG_REPORT_PACKET,
	TIMEOUT,
};

struct UdpPacket {
  uint8_t m_u8Buf[1500];
  int16_t m_i16Len;
};

static uint16_t DATA_PORT_NUMBER = 2368;     // default UDP port
static uint16_t GPS_PORT_NUMBER = 10110;     // default gps port

// Get UDP byte data via socket. For all lidar e.g. AT128 QT128
class InputSocket
{
public:
	InputSocket() {};
    ~InputSocket() { close_socket(); };
	
	/** @brief Initialize two socket object, UDP and GPS
	 *
	 *  @param deviceipaddr device ip address
	 *  @param lidarport lidar udp port number
	 *  @param gpsport gps port number
	 */
	void init(std::string deviceipaddr, std::string hostIpAddr, std::string multcastIpAddr, uint16_t lidarport = DATA_PORT_NUMBER, uint16_t gpsport = GPS_PORT_NUMBER);

	void close_socket();

	/**
	 * @brief Get a single packet via UDP socket
	 * 
	 * @param[out] pkt return udp byte data
	 * @param[in] timeout set timeout of polling data
	 * @return type of the packet, judged by the size
	 */
	PacketType getPacket(UdpPacket *&pkt, int timeout);

protected:
	uint16_t m_u16LidarPort;
	std::string m_sDeviceIpAddr;
	std::string m_sMultcastIpAddr;
	std::string m_sUdpVresion;
	bool m_bGetUdpVersion;
	int m_iTimestampIndex;
	int m_iUtcIindex;
	int m_iSequenceNumberIndex;
	int m_iPacketSize;

	int m_iSockfd;
	int m_iSockGpsfd;
	int m_iSocktNumber;
	uint32_t m_u32Sequencenum;
};
#endif // __PANDAR_INPUT_H
