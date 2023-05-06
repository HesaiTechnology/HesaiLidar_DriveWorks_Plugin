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

#include <dw/sensors/plugins/lidar/LidarDecoder.h>
#include <dw/sensors/plugins/lidar/LidarPlugin.h>

#include "HesaiLidar.h"
#include "Version.h"

using std::cout;
using std::endl;

// To store several lidar objects
std::vector<std::unique_ptr<dw::plugins::lidar::HesaiLidar>> dw::plugins::lidar::HesaiLidar::g_sensorContext;

static bool checkValid(dw::plugins::lidar::HesaiLidar* sensor)
{
    for (auto& i : dw::plugins::lidar::HesaiLidar::g_sensorContext)
    {
        if (i.get() == sensor)
            return true;
    }
    return false;
}

extern "C" {

// Drivework platform itself manages the customized sensor object 'sensor', the object won't be delete unless the platform software exit
// Transfer the pointer of your customized sensor object 'HesaiLidar' to the drivework platform pointer 'sensor'
// Params from user terminal is loaded after the initialization
dwStatus _dwSensorPlugin_createHandle(dwSensorPluginSensorHandle_t* sensor, dwSensorPluginProperties* /*properties*/,
                                      const char* params, dwContextHandle_t ctx)
{
    // cout << "_dwSensorPlugin_createHandle: " << endl;
    cout << "HESAI LIDAR PLUGIN V" << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_TINY << endl;
    if (!sensor) {
        return DW_INVALID_ARGUMENT;
    }

    size_t slotSize = 10; // Size of memory pool to read raw data from the sensor
    std::unique_ptr<dw::plugins::lidar::HesaiLidar> sensorContext(new dw::plugins::lidar::HesaiLidar(ctx, DW_NULL_HANDLE, slotSize));
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

////////////////////////////////Only for live sensor////////////////////////////////

// Attention! The virtual lidar will not call this API, so params can not be load here
// To build the TCP and UDP communication for the live sensor
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

// There may be multiple calls to dwSensorPlugin_readRawData() before a call to dwSensorPlugin_returnRawData(). 
// Won't be called by the virtual sensor playing the recorded bin file
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


// Won't be called by the virtual sensor when playing the recorded bin file
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

////////////////////////////////For virtual and live sensor////////////////////////////////

// Enqueue byte data in global buffer for next parsing. Also for virtual sensor
// For live sensor, the byte data is acquired by the three API above-mentioned
// For virtual sensor, the drivework platform is in charge
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

// To parse the UDP packet both for virtual and live sensor
// Return nvidia-format point cloud data through pointer 'output'
dwStatus _dwSensorLidarPlugin_parseDataBuffer(dwLidarDecodedPacket* output, const long int hostTimeStamp,
                                                        dwSensorPluginSensorHandle_t sensor)
{
    // std::cout << "_dwSensorLidarPlugin_parseDataBuffer：" << "hostTimeStamp=" << hostTimeStamp << std::endl;
    auto sensorContext = reinterpret_cast<dw::plugins::lidar::HesaiLidar*>(sensor);
    if (!checkValid(sensorContext))
    {
        return DW_INVALID_HANDLE;
    }
    dwStatus ret = sensorContext->parseData(output, hostTimeStamp);
    return ret;
}

// Heatbeat, To be called several times per second. Guess it checks the changes of lidar configuration
// Return lidar constants, also called by the virtual sensor
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

// Transfer the function pointers we defined to an overall object created by nvidia drivework
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