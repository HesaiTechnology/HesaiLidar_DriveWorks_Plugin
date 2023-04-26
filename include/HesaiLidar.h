/////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright [2022] [Hesai Technology Co., Ltd] 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef HESAI_LIDAR_H
#define HESAI_LIDAR_H

#include <iostream>
#include <unordered_map>
#include <fstream>

#include <BufferPool.hpp>
#include <ByteQueue.hpp>

#include "tcp_command_client.h"
#include "GeneralParser.h"
#include "InputSocket.h"

namespace dw
{
namespace plugins
{
namespace lidar
{

#define HS_LIDAR_PLUGIN_VERSION "HESAI LIDAR PLUGIN V1.1.4"

const std::string LIDAR_TYPE_AT128 = "AT128E2X";
const std::string LIDAR_TYPE_QT128 = "QT128C2X";
const std::string LIDAR_TYPE_P128 = "Pandar128E3X";

const size_t SAMPLE_BUFFER_POOL_SIZE = 5;

const uint32_t PACKET_OFFSET   = sizeof(uint32_t) + sizeof(dwTime_t);
const uint32_t RAW_PACKET_SIZE = sizeof(UdpPacket) + PACKET_OFFSET;

typedef struct
{
    uint8_t rawData[RAW_PACKET_SIZE];
} rawPacket;

class HesaiLidar
{
public:
    HesaiLidar(dwContextHandle_t ctx, dwSensorHandle_t lidarSensor, size_t slotSize)
        : m_ctx(ctx)
        , m_sal(nullptr)
        , m_lidarSensor(lidarSensor)
        , m_virtualSensorFlag(true)
        , m_buffer(sizeof(UdpPacket))
        , m_slotSize(slotSize)
    {
        resetSlot();
        count = 0;
    }

    virtual ~HesaiLidar();

    /**
     * @brief Factory method pattern: create the relevant udp parser object according to the device type.
     * 
     * @param devicetype decide which hesai lidar is used. e.g. 'device=HESAI_AT128' 'device=HESAI_QT128'
     */
    dwStatus createParser(std::string devicetype);
    
    /**
     * @brief To create a TCP client and a UDP client, maybe GPS if it exist, and init their port.
     * Only for live sensor to configure the TCP&UDP connection. Won't be used by the virtual sensor
     * 
     * @param[in] sal 
     * @param[in] params No used as the params was tranferred in the last function 'createHandle'
     */
    dwStatus createSensor(dwSALHandle_t sal, const char* params);

    dwStatus releaseSensor();

    /**
     * @brief Load essential correction file from local folder or live sensor. 
     * Attention! the point cloud won't be displayed correct if no correction file is found. It will cause point cloud shaking.
     * Virtual sensor: from a local default folder
     * Live sensor: from a live sensor by PTC command. Also from a local folder if PTC command failed. PTC is the TCP protocal defined by Hesai
     */
    dwStatus startSensor();

    dwStatus stopSensor();

    dwStatus resetSensor();

    /**
     * @brief Get the raw data packet from the live sensor. Virtual sensor is unavailable
     * 
     * @param  data return one raw data packet
     * @param  size return the size of raw data packet
     * @param  timestamp return current time from the host computer
     * @param timeout_us return if it failed to obtain correct UDP packet
    */
    virtual dwStatus readRawData(const uint8_t** data, size_t* size, dwTime_t* timestamp, dwTime_t timeout_us);

    /**
     * @brief Put the raw data packet to STL container dequeue, thread safe
     * Packet will be taken out later by API 'readRawData' for parsing
     * 
     * @param data return raw sensor data
     */
    dwStatus returnRawData(const uint8_t* data);

    /**
     * @brief Enqueue one sensor packet to a local packet buffer. For both two sensor type.
     * 
     * @param data sensor data to be enqueued to the buffer
     * @param size size of the sensor data
     * @param lenPushed return the size of sensor data
     */
    dwStatus pushData(const uint8_t* data, const size_t size, size_t* lenPushed);

    /**
     * @brief Decode sensor data from local buffer queue with typical hesai parser object
     * Typical data parser is initialized in function 'createParser'
     * 
     * @param[out] output return decoded packet to platform. 'Loading' Error occurs if the format is incorrect.
     * @param[in] hostTimeStamp Transfer host timestamp to output packet
     */
    dwStatus parseData(dwLidarDecodedPacket* output, const uint64_t hostTimeStamp);

    /**
     * @brief Deocde essential user params from the terminal, and initialize the auguments of the class
     * 
     * @param params user params from the terminal
     */
    dwStatus loadUserParams(const char* params);
    
    /**
     * @brief Load correction file.
     * 
     * Decode the correction file with typical parser object.
     * For virtual sensor: from a local file path
     * For live sensor: through TCP/PTC command
     */
    dwStatus loadLidarCorrection();
    dwStatus loadChannelConfig();
    dwStatus loadFiretimes();

    /**
     * @brief Get lidar constants
     * 
     * @param[out] constants return lidar constants, e.g. FOV
     * @return dwStatus
     */
    virtual dwStatus getDecoderConstants(_dwSensorLidarDecoder_constants* constants);

    static std::vector<std::unique_ptr<dw::plugins::lidar::HesaiLidar>> g_sensorContext;

    std::string getLidarType();

protected:
    void resetSlot();

    inline bool isVirtualSensor()
    {
        return m_virtualSensorFlag;
    }

    inline float64_t deg2Rad(float64_t deg)
    {
        return deg * 0.01745329251994329575;
    }

    inline float64_t rad2Deg(float64_t rad)
    {
        return rad * 57.29577951308232087721;
    }

    std::string getSearchString(std::string params, const std::string search);

    void printLidarProperty(dwLidarProperties* property);

    // Refer to other nvidia plugins, e.g. radar, camera
    dwContextHandle_t m_ctx      = nullptr;
    dwSALHandle_t m_sal          = nullptr;
    dwSensorHandle_t m_lidarSensor = nullptr;
    // Virtual sensor, namely playing a local bin file
    bool m_virtualSensorFlag;

    // Store UDP data in a local buffer
    dw::plugin::common::ByteQueue m_buffer;
    std::unique_ptr<dw::plugins::common::BufferPool<rawPacket>> m_slot;
    std::unordered_map<uint8_t*, rawPacket*> m_map;
    size_t m_slotSize;

    // PTC/TCP client to acuqure the correction file
    void *m_pTcpCommandClient;
    // Socket client to acquire the UDP packet
    InputSocket m_inputSocket;

    std::string m_ipAddress;
    std::string m_hostIpAddress;
    std::string m_multcastIpAddress;
    // Type of hesai lidar, default AT128
    std::string m_lidarType = LIDAR_TYPE_AT128;
    // Must be CUSTOM_EX demanded by drivework platform
    std::string m_deviceStr = "CUSTOM_EX";
    // To be updated by user through terminal input
    std::string m_correctionFilePath = "./correction_at128.dat";
    std::string m_firetimesPath = "./firetimes_qt128.dat";
    std::string m_channelConfigPath = "./channelconfig_qt128.dat";

    unsigned short m_udpPort;
    unsigned short m_ptcPort;

    // Base class pointer to be initialized as typical parser
    GeneralParser* m_Parser;
    dwLidarPointXYZI m_pointXYZI[21000][256];
    dwLidarPointRTHI m_pointRTHI[21000][256];
    // record how many points in above buffer
    int count = 0;

};

} // namespace lidar
} // namespace plugins
} // namespace dw

#endif // HESAI_LIDAR_H