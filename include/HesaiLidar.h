/**
 * @file
 * <b>HESAI Lidar: 禾赛雷达的基类</b>
 *
 * @b Description: GeneralParser.h中包含了NVIDIA DW中使用到的头文件，
 *                 ByteQueue.hpp文件将其中函数定义为inline才能避免mulit definition的报错
 */

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

#define HS_LIDAR_PLUGIN_VERSION "HESAI LIDAR PLUGIN V1.1.3"

const std::string LIDAR_TYPE_AT128 = "AT128E2X";
const std::string LIDAR_TYPE_QT128 = "QT128C2X";

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
     * @brief 根据传入的设备类型，创建一个特定的雷达数据解析器
     * 
     * @param devicetype 
     * @return dwStatus 
     */
    dwStatus createParser(std::string devicetype);
    
    /**
     * @brief 可通用！ 从传入参数串中获取通讯信息，切记，在虚拟雷达此条不会被调用
     * 创建一个TCP连接的客户端对象，初始化UDP和GPS的socket端口
     * 
     * @param[in] sal 
     * @param[in] params 
     * @return dwStatus 
     */
    dwStatus createSensor(dwSALHandle_t sal, const char* params);

    dwStatus releaseSensor();

    /**
     * @brief 加载角度文件-两种类型的雷达均需要
     * 虚拟雷达：从本地获取
     * 实时雷达：从建立的TCP连接socket中通过PTC指令获取
     * 
     * @return dwStatus 
     */
    dwStatus startSensor();

    dwStatus stopSensor();

    dwStatus resetSensor();

    virtual dwStatus readRawData(const uint8_t** data, size_t* size, dwTime_t* timestamp, dwTime_t timeout_us);

    /**
     * @brief 通用
     * 
     * @param data 
     * @return dwStatus 
     */
    dwStatus returnRawData(const uint8_t* data);

    /**
     * @brief 通用, 将传输层获得的数据缓存取出用于解析
     * 
     * @param data 
     * @param size 
     * @param lenPushed 
     * @return dwStatus 
     */
    dwStatus pushData(const uint8_t* data, const size_t size, size_t* lenPushed);

    /**
     * @brief 解析流程一样，仅里面用到的Parser不一样
     * 
     * @param[out] output 传出点云包，如果格式不合要求出现Loading情况
     * @param[in] hostTimeStamp 
     * @return dwStatus 
     */
    dwStatus parseData(dwLidarDecodedPacket* output, const uint64_t hostTimeStamp);

    /**
     * @brief 解析用户传入字符串，初始化类的公共参数
     * 
     * @param params 用户传入的字符串
     * @return dwStatus 
     */
    dwStatus loadUserParams(const char* params);
    
    /**
     * @brief 获取一次雷达的校正文件，实时雷达建立TCP连接通过PTC指令获取，虚拟雷达直接从本地当前路径读取
     * 也使用Parser做了角度文件的解析，应当分开， 各雷达均需要
     * 命名Calibration更合适
     */
    dwStatus loadLidarCorrection();

    dwStatus loadChannelConfig();
    dwStatus loadFiretimes();

    /**
     * @brief 传入雷达的参数，如俯仰角等信息
     * 
     * @param[out] constants 将参数传入
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

    /**
     * @brief 从用户输入字符串序列中获取搜寻的参数
     * 
     * @param params 用户输入字符串序列
     * @param search 搜寻的参数
     * @return std::string 找到的参数
     */
    std::string getSearchString(std::string params, const std::string search);

    void printLidarProperty(dwLidarProperties* property);

    // 参考另三Plugin
    dwContextHandle_t m_ctx      = nullptr;
    dwSALHandle_t m_sal          = nullptr;
    dwSensorHandle_t m_lidarSensor = nullptr;
    bool m_virtualSensorFlag;

    dw::plugin::common::ByteQueue m_buffer;
    std::unique_ptr<dw::plugins::common::BufferPool<rawPacket>> m_slot;
    std::unordered_map<uint8_t*, rawPacket*> m_map;
    size_t m_slotSize;

    
    // PTC获取各种校正文件的TCP
    void *m_pTcpCommandClient;
    // 获取UDP包
    InputSocket m_inputSocket;

    std::string m_ipAddress;
    std::string m_hostIpAddress;
    std::string m_multcastIpAddress;
    // 禾赛雷达的类型，默认AT128
    std::string m_lidarType = LIDAR_TYPE_AT128;
    // ！！！ CUSTOM_EX，据说乱传会出错 YES YES YES
    std::string m_deviceStr = "CUSTOM_EX";
    // 子类中需要改写这一路径, 此处为一默认值
    std::string m_correctionFilePath = "./correction_at128.dat";
    std::string m_firetimesPath = "./firetimes_qt128.dat";
    std::string m_channelConfigPath = "./channelconfig_qt128.dat";

    unsigned short m_udpPort;
    unsigned short m_ptcPort;

    // 基类指针-在子类中初始化为对应的 Udp4_3_Parser
    GeneralParser* m_Parser;
    dwLidarPointXYZI m_pointXYZI[21000][256];
    dwLidarPointRTHI m_pointRTHI[21000][256];
    int count;

};

} // namespace lidar
} // namespace plugins
} // namespace dw

#endif // HESAI_LIDAR_H