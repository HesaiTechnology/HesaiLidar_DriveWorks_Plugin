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

// Angle unit = 1/1000 360000/1000 = 360, the same as unit in correction file
#define HS_LIDAR_QT128_AZIMUTH_SIZE (360000)
#define HS_LIDAR_QT128_AZIMUTH_UNIT (1000)
#define HS_LIDAR_QT128_LASER_NUM (128)
#define HS_LIDAR_QT128_LOOP_NUM (4)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG (0.0354)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT (-0.0072)

#include "GeneralParser.h"
#include "HsLidarQTV2.h"

// Default channel config is the common use of QT128, using all 128 channels
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
   * @brief Get vertical angle of each laser channel
   * 
   * @param channel 0-127 each laser channel, QT128 has 128
   * @return int16_t 1.223 degree will return 1223，unit = 1/1000
   */
  int16_t GetVecticalAngle(int channel) override;

  virtual int LoadFiretimesString(const char *firetimes) override;
  virtual void LoadFiretimesFile(std::string firetimes_path) override;
  
  /**
   * @brief Initialize the m_PandarQTChannelConfig, specialized for QT
   */
  virtual int LoadChannelConfigString(const char *channelconfig) override;
  virtual void LoadChannelConfigFile(std::string channel_config_path) override;

 private:
  virtual double GetFiretimesCorrection(int laserId, double speed, int loopIndex);

  // To be initialized by func 'LoadFiretimesString'
  std::array<std::array<float, HS_LIDAR_QT128_LASER_NUM>, HS_LIDAR_QT128_LOOP_NUM> m_vQT128Firetime;
  // To be initialized by func 'LoadChannelConfigString'
  PandarQTChannelConfig m_PandarQTChannelConfig;
  
  // Only for QT128 and etc，not for AT128
  std::vector<double> m_vFiretimeCorrection;
};

#endif  // UDP3_2_PARSER_H_
