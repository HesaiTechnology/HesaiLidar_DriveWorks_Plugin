#include <dw/sensors/plugins/lidar/LidarDecoder.h>
#include <dw/sensors/plugins/lidar/LidarPlugin.h>

#include "HesaiLidar.h"

using std::cout;
using std::endl;

// 这里面include的，添加了一个<BufferPool.hpp> <ByteQueue.hpp>
// 他们的hpp是聚集了.h和.cpp的，因此

// 该数组为static类型，类内存在，不过是一个存的数组，取了这么长一个名字？
std::vector<std::unique_ptr<dw::plugins::lidar::HesaiLidar>> dw::plugins::lidar::HesaiLidar::g_sensorContext;

//#######################################################################################
static bool checkValid(dw::plugins::lidar::HesaiLidar* sensor)
{
    for (auto& i : dw::plugins::lidar::HesaiLidar::g_sensorContext)
    {
        if (i.get() == sensor)
            return true;
    }
    return false;
}

// exported functions
extern "C" {

//#######################################################################################
// 该函数后即可进行数据decode, 将new创建的HesaiLidar对象的指针传给dwSensorPluginSensorHandle_t
// 内部维护dwSensorPluginSensorHandle_t，所有的操作均是对这个指针对象sensor
dwStatus _dwSensorPlugin_createHandle(dwSensorPluginSensorHandle_t* sensor, dwSensorPluginProperties* /*properties*/,
                                      const char* params, dwContextHandle_t ctx)
{
    cout << HS_LIDAR_PLUGIN_VERSION << endl;
    // cout << "_dwSensorPlugin_createHandle: " << endl;
    if (!sensor) {
        return DW_INVALID_ARGUMENT;
    }

    size_t slotSize = 10; // Size of memory pool to read raw data from the sensor
    // 根据获取到的device=HESAI_AT128还是HESAI_AT128 new出对应的
    std::unique_ptr<dw::plugins::lidar::HesaiLidar> sensorContext(new dw::plugins::lidar::HesaiLidar(ctx, DW_NULL_HANDLE, slotSize));
    // (void) params;
    sensorContext->loadUserParams(params);

    std::string lidartype = sensorContext->getLidarType();
    if (sensorContext->createParser(lidartype) != DW_SUCCESS) {
        cout << "_dwSensorPlugin_createHandle: createParser Error" << endl;
        return DW_CANNOT_CREATE_OBJECT;
    }

    dw::plugins::lidar::HesaiLidar::g_sensorContext.push_back(std::move(sensorContext));
    *sensor = dw::plugins::lidar::HesaiLidar::g_sensorContext.back().get();

    return DW_SUCCESS;
}

//#######################################################################################
// 如果是虚拟雷达，不会调用此函数，因此不能获得decode_path=等一系列输入
// 雷达的通讯与信号传输，reinterpret_cast又将Handle转回HesaiLidar对象！
// 仅用于实时雷达，虚拟雷达不调用，用户的输入params根本没机会获取，只执行一次
dwStatus _dwSensorPlugin_createSensor(const char* params, dwSALHandle_t sal, dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorPlugin_createSensor: " << std::endl;

    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->createSensor(sal, params);
}

//#######################################################################################
dwStatus _dwSensorPlugin_start(dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorPlugin_start: " << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->startSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_release(dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorPlugin_release: " << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    for (auto iter =
             dw::plugins::lidar::HesaiLidar::g_sensorContext.begin();
         iter != dw::plugins::lidar::HesaiLidar::g_sensorContext.end();
         ++iter)
    {
        if ((*iter).get() == sensor)
        {
            sensorContext->stopSensor();
            sensorContext->releaseSensor();
            dw::plugins::lidar::HesaiLidar::g_sensorContext.erase(iter);
            return DW_SUCCESS;
        }
    }
    return DW_FAILURE;
}

//#######################################################################################
dwStatus _dwSensorPlugin_stop(dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorPlugin_stop: " << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->stopSensor();
}

//#######################################################################################
dwStatus _dwSensorPlugin_reset(dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorPlugin_reset: " << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->resetSensor();
}

//#######################################################################################
// There may be multiple calls to dwSensorPlugin_readRawData() before a call to dwSensorPlugin_returnRawData(). 
// 虚拟雷达点云播放不调用此接口-反复执行 _dwSensorPlugin_readRawData _dwSensorPlugin_returnRawData
dwStatus _dwSensorPlugin_readRawData(const uint8_t** data, size_t* size, dwTime_t* timestamp,
                                     dwTime_t timeout_us, dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorPlugin_readRawData" << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->readRawData(data, size, timestamp, timeout_us);
}

//#######################################################################################
dwStatus _dwSensorPlugin_returnRawData(const uint8_t* data, dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorPlugin_returnRawData: " << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->returnRawData(data);
}

//#######################################################################################
// 前几个函数已经获得的点云包，bin文件中的也是，此处解析
// 虚拟雷达也调用-反复执行， 如下三个函数是将UDP包解析到NVIDIA要求的格式
dwStatus _dwSensorPlugin_pushData(size_t* lenPushed, const uint8_t* data, const size_t size, dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorPlugin_pushData: " << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }

    return sensorContext->pushData(data, size, lenPushed);
}

//#######################################################################################
// 虚拟雷达也调用
dwStatus _dwSensorLidarPlugin_parseDataBuffer(dwLidarDecodedPacket* output, const long int hostTimeStamp,
                                                        dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorLidarPlugin_parseDataBuffer时间戳：" << hostTimeStamp << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }
    dwStatus ret = sensorContext->parseData(output, hostTimeStamp);
    return ret;
}

//######################################################################################
// 虚拟雷达也调用,此时每次都会调用，？？为何每次解析都会调用？可能防止常数发生变化也能更新？
// 心跳，每秒会调用几次
dwStatus _dwSensorLidarPlugin_getDecoderConstants(_dwSensorLidarDecoder_constants* constants, dwSensorPluginSensorHandle_t sensor) 
{
    // std::cout << "_dwSensorLidarPlugin_getDecoderConstants" << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }
    dwStatus ret = sensorContext->getDecoderConstants(constants);
    
    return ret;
}

//######################################################################################
dwStatus dwSensorLidarPlugin_getFunctionTable(dwSensorLidarPluginFunctionTable* functions)
{
    // std::cout << "dwSensorLidarPlugin_getFunctionTable: " << std::endl;
    if (functions == nullptr)
        return DW_INVALID_ARGUMENT;
    
    functions->common.createHandle = _dwSensorPlugin_createHandle;
    functions->common.createSensor = _dwSensorPlugin_createSensor;
    functions->common.release = _dwSensorPlugin_release;
    functions->common.start = _dwSensorPlugin_start;
    functions->common.stop = _dwSensorPlugin_stop;
    functions->common.reset = _dwSensorPlugin_reset;
    functions->common.readRawData = _dwSensorPlugin_readRawData;
    functions->common.returnRawData = _dwSensorPlugin_returnRawData;
    functions->common.pushData = _dwSensorPlugin_pushData;
    functions->parseDataBuffer = _dwSensorLidarPlugin_parseDataBuffer;
    functions->getDecoderConstants = _dwSensorLidarPlugin_getDecoderConstants;
    return DW_SUCCESS;
}

} // extern "C"