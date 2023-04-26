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

/**
 * @file
 * <b>HESAI Plugin for DriveWorks: Lidar Sensor UDP Parser</b>
 *
 * @b Description: This file defines the udp parser for QT128.
 */

#ifndef UDP3_2_PARSER_H_
#define UDP3_2_PARSER_H_

// 角度修正文件的精度是0.001, 360 * 1000获得分度, 此时只能用int_32，单位/1000
#define HS_LIDAR_QT128_AZIMUTH_SIZE (360000)
#define HS_LIDAR_QT128_AZIMUTH_UNIT (1000)
#define HS_LIDAR_QT128_LASER_NUM (128)
#define HS_LIDAR_QT128_LOOP_NUM (4)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG (0.0354)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT (-0.0072)

#include "GeneralParser.h"
#include "HsLidarQTV2.h"

// QT128的校正文件，通道校正文件, 也没啥用，全123456就和默认值一样
struct PandarQTChannelConfig {
 public:
  uint16_t m_u16Sob;
  uint8_t m_u8MajorVersion;
  uint8_t m_u8MinVersion;
  uint8_t m_u8LaserNum;
  uint8_t m_u8BlockNum;
  std::vector<std::vector<int>> m_vChannelConfigTable;
  std::string m_sHashValue;
  bool m_bIsChannelConfigObtained;
};

class Udp3_2_Parser : public GeneralParser {
 public:
  Udp3_2_Parser();
  virtual ~Udp3_2_Parser();

  dwStatus GetDecoderConstants(_dwSensorLidarDecoder_constants* constants) override;

  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) override;     

  /**
   * @brief 获取每线对应的垂直方位角
   * 
   * @param channel 0-127 QT128有128线
   * @return int16_t 1.223度返回值为 1223，单位/1000
   */
  int16_t GetVecticalAngle(int channel) override;

  /**
   * @brief 为了初始化m_vQT128Firetime
   */
  virtual int LoadFiretimesString(const char *firetimes) override;
  virtual void LoadFiretimesFile(std::string firetimes_path) override;
  
  /**
   * @brief 初始化m_PandarQTChannelConfig QT雷达特有
   * 
   * @param[in] channelconfig 实时雷达获取的字节流，或从文本中读取的 
   * @return int 0成功 -1失败
   */
  virtual int LoadChannelConfigString(const char *channelconfig) override;
  virtual void LoadChannelConfigFile(std::string channel_config_path) override;

 private:
  /**
   * @brief 计算效率上 必须返回int类型
   * 
   * @param laserId 雷达线数 0-127
   * @param speed 雷达转速
   * @param loopIndex ？？从包尾获取，与回波模式两参数相关
   * @return double 
   */
  virtual double GetFiretimesCorrection(int laserId, double speed, int loopIndex);

  // 需通过函数初始化：LoadFiretimesString
  std::array<std::array<float, HS_LIDAR_QT128_LASER_NUM>, HS_LIDAR_QT128_LOOP_NUM> m_vQT128Firetime;
  // 需通过函数初始化：LoadChannelConfigString
  PandarQTChannelConfig m_PandarQTChannelConfig;
  
  std::array<float, HS_LIDAR_QT128_AZIMUTH_SIZE> sin_map;
  std::array<float, HS_LIDAR_QT128_AZIMUTH_SIZE> cos_map;

  // Only for QT128 and etc，not for AT128
  std::vector<double> m_vFiretimeCorrection;
};

#endif  // UDP3_2_PARSER_H_
