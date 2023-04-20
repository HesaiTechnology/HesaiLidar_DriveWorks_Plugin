/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef SRC_TCP_COMMAND_CLIENT_H_
#define SRC_TCP_COMMAND_CLIENT_H_

#ifdef __cplusplus
extern "C" {
#endif

//AT128和QT128均是发这个指令， PTC_COMMAND_GET_LIDAR_CALIBRATION=5
typedef enum {
  PTC_COMMAND_GET_CALIBRATION = 0,
  PTC_COMMAND_SET_CALIBRATION,
  PTC_COMMAND_HEARTBEAT,
  PTC_COMMAND_RESET_CALIBRATION,
  PTC_COMMAND_TEST,
  PTC_COMMAND_GET_LIDAR_CALIBRATION,
  PTC_COMMAND_GET_LIDAR_CONFIG_INFO = 8,
  PTC_COMMAND_GET_LIDAR_STATUS = 9,
  PTC_COMMAND_SET_LIDAR_SPIN_RATE = 23,
  PTC_COMMAND_SET_LIDAR_OPERATE_MODE = 28,
  PTC_COMMAND_SET_LIDAR_RETURN_MODE = 30,
  PTC_COMMAND_SET_LIDAR_LENS_HEAT_SWITCH = 147,
  PTC_COMMAND_GET_LIDAR_LENS_HEAT_SWITCH = 148,
  PTC_COMMAND_GET_LIDAR_CHANNEL_CONFIG = 0xA8,
  PTC_COMMAND_GET_LIDAR_FIRETIMES = 0xA9,
} PTC_COMMAND;

typedef enum {
  PTC_ERROR_NO_ERROR = 0,
  PTC_ERROR_BAD_PARAMETER,
  PTC_ERROR_CONNECT_SERVER_FAILED,
  PTC_ERROR_TRANSFER_FAILED,
  PTC_ERROR_NO_MEMORY,
} PTC_ErrCode;

typedef enum {
  CERTIFY_MODE_NONE = 0,
  CERTIFY_MODE_SINGLE,
  CERTIFY_MODE_DUAL,
  CERTIFY_MODE_ERROR,
} CERTIFY_MODE;

typedef struct TcpCommandHeader_s {
  unsigned char cmd;
  unsigned char ret_code;
  unsigned int len;
} TcpCommandHeader;

typedef struct TC_Command_s {
  TcpCommandHeader header;
  unsigned char* data;

  unsigned char* ret_data;
  unsigned int ret_size;
} TC_Command;

/**
 * @brief 创建一个TCP连接
 * 
 * @param ip 地址
 * @param port 端口
 * @return void* 创建的TCP客户端对象指针
 */
void* TcpCommandClientNew(const char* ip, const unsigned short port);
void BuildCmd(TC_Command command, PTC_COMMAND cmd, unsigned char* data);
PTC_ErrCode TcpCommandSetCalibration(const void* handle, const unsigned char* buffer,
                                     unsigned int len);
PTC_ErrCode TcpCommandGetCalibration(const void* handle, char** buffer,
                                     unsigned int* len);
/**
 * @brief 从TCP连接的socket中取得校正文件的字节流, 只需要获取一次即可！
 * 
 * @param handle TcpCommandClient，结构体，保存文件描述符和线程标识
 * @param[out] buffer 读到的校正文件字节流
 * @param[out] len 校正文件字节流长度
 * @return PTC_ErrCode =0 为正常
 */
PTC_ErrCode TcpCommandGetLidarCalibration(const void* handle, unsigned char** buffer,
                                          unsigned int* len);
PTC_ErrCode TcpCommandGetLidarLensHeatSwitch(const void* handle, unsigned char** buffer,
                                          unsigned int* len); 
PTC_ErrCode TcpCommandGetLidarStatus(const void* handle, unsigned char** buffer,
                                          unsigned int* len);  
PTC_ErrCode TcpCommandGetLidarConfigInfo(const void* handle, unsigned char** buffer,
                                          unsigned int* len);                                                                                                                           
PTC_ErrCode TcpCommandResetCalibration(const void* handle);
PTC_ErrCode TcpCommandSetLidarStandbyMode(const void* handle);
PTC_ErrCode TcpCommandSetLidarNormalMode(const void* handle);
PTC_ErrCode TcpCommandSetLidarLensHeatSwitch(const void* handle, uint8_t heatSwitch);
PTC_ErrCode TcpCommandSetLidarReturnMode(const void* handle, uint8_t mode);
PTC_ErrCode TcpCommandSetLidarSpinRate(const void* handle, uint16_t spinRate);
PTC_ErrCode TcpCommandSet(const void* handle, PTC_COMMAND cmd, unsigned char* data, uint32_t len);
PTC_ErrCode TcpCommandGet(const void* handle, PTC_COMMAND cmd, unsigned char** buffer, unsigned int* len);
void TcpCommandClientDestroy(const void* handle);

#ifdef __cplusplus
}
#endif

#endif  //  SRC_TCP_COMMAND_CLIENT_H_
