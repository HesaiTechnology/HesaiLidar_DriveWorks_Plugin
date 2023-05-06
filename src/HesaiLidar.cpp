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

#include <thread>
#include <chrono>
#include "HesaiLidar.h"
#include "Udp4_3_Parser.h"
#include "Udp3_2_Parser.h"
#include "Udp1_4_Parser.h"

namespace dw
{
namespace plugins
{
namespace lidar
{

HesaiLidar::~HesaiLidar() {
    if (m_Parser != nullptr) {
        delete m_Parser;
        m_Parser = nullptr;
    }
}

dwStatus HesaiLidar::createParser(std::string lidartype) {
    // std::cout << "HesaiLidar::createParser" << std::endl;
    if (lidartype == LIDAR_TYPE_AT128) {
        m_Parser = new Udp4_3_Parser();
    } else if (lidartype == LIDAR_TYPE_QT128) {
        m_Parser = new Udp3_2_Parser();
    } else if (lidartype == LIDAR_TYPE_P128) {
        m_Parser = new Udp1_4_Parser();
    } else {
        std::cout << "createParser, create specific parser Error, lidartype=" << lidartype << std::endl;
        return DW_CANNOT_CREATE_OBJECT;
    }

    return DW_SUCCESS;
}

dwStatus HesaiLidar::createSensor(dwSALHandle_t sal, const char* params)
{
    m_sal               = sal;
    m_virtualSensorFlag = false;
    (void) params;
    m_pTcpCommandClient = TcpCommandClientNew(m_ipAddress.c_str(), m_ptcPort);  
             
    return DW_SUCCESS;
}

dwStatus HesaiLidar::releaseSensor()
{
    if (!isVirtualSensor()) {
        m_inputSocket.CloseSocket();
    }
    return DW_SUCCESS;
}

dwStatus HesaiLidar::startSensor()
{
    // std::cout << "HesaiLidar::startSensor, loading correction files" << std::endl;
    if (!isVirtualSensor()) {
        m_inputSocket.InitSocket(m_ipAddress, m_hostIpAddress, m_multcastIpAddress, m_udpPort); 
    }
    
    if (loadLidarCorrection() != DW_SUCCESS) {
        std::cout << "startSensor: first loading calibration Error, try local file" << std::endl;
        if (m_Parser->LoadCorrectionFile(m_correctionFilePath) != 0) {
            // return DW_FAILURE;
        }
    }

    if (m_lidarType == LIDAR_TYPE_QT128) {
        // TODO support channel config or firetime config
        // if(loadChannelConfig() != DW_SUCCESS) {
        //     std::cout << "startSensor: QT128 loadChannelConfig Error" << std::endl;
        //     // return DW_FAILURE;
        // }
        // if(loadFiretimes() != DW_SUCCESS) {
        //     std::cout << "startSensor: QT128 loadFiretimes Error" << std::endl;
        //     // return DW_FAILURE;
        // }
    }

    return DW_SUCCESS;
}

dwStatus HesaiLidar::stopSensor()
{
    if (!isVirtualSensor()) {
        m_inputSocket.CloseSocket();
    }    
    return DW_SUCCESS;
}

dwStatus HesaiLidar::resetSensor()
{
    m_buffer.clear();
    resetSlot();

    // if (!isVirtualSensor())
    //     m_inputSocket.CloseSocket();

    return DW_SUCCESS;
}

dwStatus HesaiLidar::readRawData(const uint8_t** data, size_t* size, dwTime_t* timestamp, dwTime_t timeout_us)
{
    if (!isVirtualSensor()) {
        rawPacket* result = nullptr;
        bool ok           = m_slot->get(result);
        if (!ok)
        {
            std::cerr << "readRawData: Read raw data, slot not empty\n";
            return DW_BUFFER_FULL;
        }
        UdpPacket* packet = reinterpret_cast<UdpPacket*>(&(result->rawData[PACKET_OFFSET]));
        while (1)
        {
            PacketType type = m_inputSocket.GetPacket(packet, timeout_us / 1000);
            if (type ==  POINTCLOUD_PACKET) break;
            if (type ==  TIMEOUT) return DW_TIME_OUT;
            usleep(10);
        }
        dwContext_getCurrentTime(timestamp, m_ctx);
        uint32_t rawDataSize = sizeof(UdpPacket);
        memcpy(&result->rawData[0], &rawDataSize, sizeof(uint32_t));
        memcpy(&result->rawData[sizeof(uint32_t)], timestamp, sizeof(dwTime_t));

        *data = &(result->rawData[0]);
        *size = RAW_PACKET_SIZE - 12;
        return DW_SUCCESS;
    }

    return DW_SUCCESS;
}

dwStatus HesaiLidar::returnRawData(const uint8_t* data)
{
    if (data == nullptr)
    {
        return DW_INVALID_HANDLE;
    }

    bool ok = m_slot->put(const_cast<rawPacket*>(m_map[const_cast<uint8_t*>(data)]));
    if (!ok)
    {
        std::cerr << "returnRawData: LidarPlugin return raw data, invalid data pointer"
                    << std::endl;
        return DW_INVALID_ARGUMENT;
    }

    data = nullptr;

    return DW_SUCCESS;
}

dwStatus HesaiLidar::pushData(const uint8_t* data, const size_t size, size_t* lenPushed)
{
    if (data == nullptr || lenPushed == nullptr)
    {
        return DW_INVALID_HANDLE;
    }
    m_buffer.enqueue(data, size);
    *lenPushed = size;

    return DW_SUCCESS;
}

dwStatus HesaiLidar::parseData(dwLidarDecodedPacket* output, const uint64_t hostTimeStamp)
{
    const UdpPacket* msg;
    // Peek the first packet from the buffer queue
    if (!m_buffer.peek(reinterpret_cast<const uint8_t**>(&msg)))
    {
        return DW_FAILURE;
    }

    if (output == nullptr)
    {
        return DW_INVALID_HANDLE;
    }
    count++;
    m_Parser->ParserOnePacket(output, msg->m_u8Buf, msg->m_i16Len, m_pointXYZI[count], m_pointRTHI[count]);
    dwContext_getCurrentTime(&output->hostTimestamp, m_ctx);
    m_buffer.dequeue();
    
    output->hostTimestamp = hostTimeStamp;
    if (count > 20000) count = 0;
    // m_Parser->PrintDwPoint(&output->pointsXYZI[0]);

    return DW_SUCCESS;
}

dwStatus HesaiLidar::loadLidarCorrection()
{
    // std::cout << "HesaiLidar::loadLidarCorrection" << std::endl;
    if (isVirtualSensor()) {
        int ret = m_Parser->LoadCorrectionFile(m_correctionFilePath);
        return ret == 0 ? DW_SUCCESS : DW_SAL_CANNOT_INITIALIZE;
    } else {
        if(m_pTcpCommandClient == NULL) {
            printf("loadLidarCorrection: m_pTcpCommandClient is Null\n");
            return DW_CANNOT_CREATE_OBJECT;
        }
        unsigned char *buffer = NULL;
        unsigned int len = 0;
        PTC_ErrCode status = TcpCommandGetLidarCalibration(m_pTcpCommandClient, &buffer, &len);
        if (status != PTC_ERROR_NO_ERROR || buffer == NULL) {
            printf("loadLidarCorrection: Get calibration file Error,check ptc connection\n");
            return DW_SAL_CANNOT_INITIALIZE;
        }
        
        int ret = m_Parser->ParseCorrectionString((char*)buffer);
        free(buffer);
        if(ret != 0) {
            printf("loadLidarCorrection: correction file from PTC parsing Error \n");
            return DW_SAL_CANNOT_INITIALIZE;
        }
    }
    
    return DW_SUCCESS;
}

dwStatus HesaiLidar::loadChannelConfig() {
    if (isVirtualSensor() == true) {
        m_Parser->LoadChannelConfigFile(m_channelConfigPath);
    } else {
        unsigned char* buffer = NULL;
        unsigned int len = 0;
        PTC_ErrCode status = TcpCommandGet(m_pTcpCommandClient, PTC_COMMAND_GET_LIDAR_CHANNEL_CONFIG, &buffer, &len);
        if (status != PTC_ERROR_NO_ERROR) {
            printf("loadChannelConfig: Parse Lidar Correction Error,check ptc connection\n");
            return DW_CANNOT_CREATE_OBJECT;
        } else {
            if (m_Parser->LoadChannelConfigString((char*) buffer) != 0) {
                printf("loadChannelConfig: Parse channel config Error,check ptc connection\n");
                return DW_CANNOT_CREATE_OBJECT;
            }
        }
    }

    return DW_SUCCESS;
}

dwStatus HesaiLidar::loadFiretimes() {
    if (isVirtualSensor() == true) {
        m_Parser->LoadFiretimesFile(m_firetimesPath);
    } else {
        unsigned char* buffer = NULL;
        unsigned int len = 0;
        PTC_ErrCode status = TcpCommandGet(m_pTcpCommandClient, PTC_COMMAND_GET_LIDAR_FIRETIMES, &buffer, &len);
        if (status != PTC_ERROR_NO_ERROR) {
            printf("loadFiretimes: Load FireTimes from lidar Error,check ptc connection\n");
            return DW_CANNOT_CREATE_OBJECT;
        } else {
            if (m_Parser->LoadFiretimesString((char*) buffer) != 0) {
                printf("loadFiretimes: Parse firetimes file Error\n");
                return DW_CANNOT_CREATE_OBJECT;
            }
        }
    }

    return DW_SUCCESS;
}

dwStatus HesaiLidar::getDecoderConstants(_dwSensorLidarDecoder_constants* constants) {
    // ! Must assign deviceString to be CUSTOM_EX, or A black screen Error might occur
    memcpy(&constants->properties.deviceString, m_deviceStr.c_str(), 256);
    m_Parser->GetDecoderConstants(constants);

    return DW_SUCCESS;
}

std::string HesaiLidar::getLidarType() {
    return m_lidarType;
}

////////////////////////////////////////privete////////////////////////////////////////

void HesaiLidar::resetSlot()
{
    m_slot = std::make_unique<dw::plugins::common::BufferPool<rawPacket>>(m_slotSize);
    m_map.clear();

    std::vector<rawPacket*> vectorOfRawPacketPtr;
    std::vector<uint8_t*> vectorOfRawDataPtr;
    for (uint8_t i = 0; i < m_slotSize; ++i)
    {
        rawPacket* rawPacketPtr = nullptr;
        bool ok                 = m_slot->get(rawPacketPtr);
        if (!ok)
        {
            std::cerr << "resetSlot(), BufferPool instanciated with non empty slots" << std::endl;
        }

        uint8_t* rawDataPtr = &(rawPacketPtr->rawData[0]);
        vectorOfRawPacketPtr.push_back(rawPacketPtr);
        vectorOfRawDataPtr.push_back(rawDataPtr);
    }

    for (uint8_t i = 0; i < m_slotSize; ++i)
    {
        rawPacket* rawPacketPtr = vectorOfRawPacketPtr[i];
        uint8_t* rawDataPtr     = vectorOfRawDataPtr[i];
        m_map[rawDataPtr]       = rawPacketPtr;
        bool ok                 = m_slot->put(rawPacketPtr);
        if (!ok)
        {
            std::cerr << "resetSlot(), BufferPool invalid put" << std::endl;
        }
    }
}

std::string HesaiLidar::getSearchString(std::string params, const std::string search) {
    std::string result = "";
    size_t pos = params.find(search);
    if (pos == std::string::npos)
    {
        // std::cout << "loadUserParams: search string is not specified, " << search << "=?" << std::endl;
    } else {
        result = params.substr(pos + search.length());
        pos                = result.find_first_of(",");
        result             = result.substr(0, pos);
    }
    
    return result;
}

dwStatus HesaiLidar::loadUserParams(const char* params) {
    // std::cout << "HesaiLidar::loadUserParams, params = " << params << std::endl;
    std::string paramsString = params;

    m_correctionFilePath = getSearchString(paramsString, "correction_file=");
    m_lidarType = getSearchString(paramsString, "lidar_type=");
    m_deviceStr = getSearchString(paramsString, "device=");

    // std::cout << "loadUserParams: device=" << m_deviceStr << ",lidar_type=" << m_lidarType
    //           << ",correction_file=" << m_correctionFilePath << std::endl;

    // connection params
    m_ipAddress    = getSearchString(paramsString, "ip=");
    m_hostIpAddress = getSearchString(paramsString, "host_ip=");
    m_multcastIpAddress = getSearchString(paramsString, "multcast_ip=");
    
    std::string retStr = getSearchString(paramsString, "udp_port=");
    if (retStr != "") {
        try{
            m_udpPort = std::stoi(retStr);
        }
        catch(const std::exception& e){
            std::cerr << "wrong param udp_port" << e.what() << '\n';
        }
    }

    retStr = getSearchString(paramsString, "ptc_port=");
    if (retStr != "") {
        try{
            m_ptcPort = std::stoi(retStr);
        }
        catch(const std::exception& e){
            std::cerr << "wrong param ptc_port" << e.what() << '\n';
        }
    }
    
    // std::cout << "ip=" << m_ipAddress << ",udp_port=" << m_udpPort << ",ptc_port=" << m_ptcPort
    //           << ",multcast_ip=" << m_multcastIpAddress << std::endl;

    return DW_SUCCESS;
}

void HesaiLidar::printLidarProperty(dwLidarProperties* property) {
    std::cout << "printLidarProperty: deviceString=" << property->deviceString
              << ", spinFrequency=" << property->spinFrequency
              << ", packetsPerSecond=" << property->packetsPerSecond
              << ", packetsPerSpin="   << property->packetsPerSpin
              << ", pointsPerSecond="  << property->pointsPerSecond
              << ", pointsPerPacket="  << property->pointsPerPacket
              << ", pointsPerSpin="    << property->pointsPerSpin
              << ", pointStride="      << property->pointStride
              << ", horizontalFOVStart=" << property->horizontalFOVStart
              << ", horizontalFOVEnd="   << property->horizontalFOVEnd
              << ", numberOfRows="       << property->numberOfRows
              << ", verticalFOVStart="   << property->verticalFOVStart
              << ", verticalFOVEnd="     << property->verticalFOVEnd << std::endl;
}

} // namespace lidar
} // namespace plugins
} // namespace dw

