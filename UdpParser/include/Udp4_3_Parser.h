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
 * @b Description: This file defines the udp parser for AT128.
 */

#ifndef UDP4_3_PARSER_H_
#define UDP4_3_PARSER_H_

#define MAX_AZI_LEN (36000 * 256)
#define CIRCLE_ANGLE (36000)
#define CORRECTION_AZIMUTH_STEP (200)
#define CORRECTION_AZIMUTH_NUM (180)
#define FINE_AZIMUTH_UNIT (256)
#define AZIMUTH_UNIT (25600.0f)
#define AT128_LASER_NUM (128)
#define PANDAR_AT128_EDGE_AZIMUTH_OFFSET (7500)
#define PANDAR_AT128_EDGE_AZIMUTH_SIZE (1600)

#include <array>
#include "GeneralParser.h"

struct PandarATCorrectionsHeader {
  uint8_t delimiter[2];
  uint8_t version[2];
  uint8_t channel_number;
  uint8_t mirror_number;
  uint8_t frame_number;
  uint8_t frame_config[8];
  uint8_t resolution;
};
static_assert(sizeof(PandarATCorrectionsHeader) == 16, "");

struct PandarATFrameInfo {
  uint32_t start_frame[8];
  uint32_t end_frame[8];
  int32_t azimuth[AT128_LASER_NUM];
  int32_t elevation[AT128_LASER_NUM];
  std::array<float, MAX_AZI_LEN> sin_map;
  std::array<float, MAX_AZI_LEN> cos_map;
};

struct PandarATCorrections {
 public:
  PandarATCorrectionsHeader header;
  uint16_t start_frame[8];
  uint16_t end_frame[8];
  int16_t azimuth[AT128_LASER_NUM];
  int16_t elevation[AT128_LASER_NUM];
  int8_t azimuth_offset[CIRCLE_ANGLE];
  int8_t elevation_offset[CIRCLE_ANGLE];
  uint8_t SHA256[32];
  PandarATFrameInfo l;  // V1.5
  std::array<float, MAX_AZI_LEN> sin_map;
  std::array<float, MAX_AZI_LEN> cos_map;
  PandarATCorrections() {
    for (int i = 0; i < MAX_AZI_LEN; ++i) {
      sin_map[i] = std::sin(2 * i * M_PI / MAX_AZI_LEN);
      cos_map[i] = std::cos(2 * i * M_PI / MAX_AZI_LEN);
    }
  }
  static const int STEP = CORRECTION_AZIMUTH_STEP;
  int8_t getAzimuthAdjust(uint8_t ch, uint16_t azi) const {
    unsigned int i = std::floor(1.f * azi / STEP);
    unsigned int l = azi - i * STEP;
    float k = 1.f * l / STEP;
    return round((1 - k) * azimuth_offset[ch * CORRECTION_AZIMUTH_NUM + i] +
                 k * azimuth_offset[ch * CORRECTION_AZIMUTH_NUM + i + 1]);
  }
  int8_t getElevationAdjust(uint8_t ch, uint16_t azi) const {
    unsigned int i = std::floor(1.f * azi / STEP);
    unsigned int l = azi - i * STEP;
    float k = 1.f * l / STEP;
    return round((1 - k) * elevation_offset[ch * CORRECTION_AZIMUTH_NUM + i] +
                 k * elevation_offset[ch * CORRECTION_AZIMUTH_NUM + i + 1]);
  }
  static const int STEP3 = CORRECTION_AZIMUTH_STEP * FINE_AZIMUTH_UNIT;
  int8_t getAzimuthAdjustV3(uint8_t ch, uint32_t azi) const {
    unsigned int i = std::floor(1.f * azi / STEP3);
    unsigned int l = azi - i * STEP3;
    float k = 1.f * l / STEP3;
    return round((1 - k) * azimuth_offset[ch * CORRECTION_AZIMUTH_NUM + i] +
                 k * azimuth_offset[ch * CORRECTION_AZIMUTH_NUM + i + 1]);
  }
  int8_t getElevationAdjustV3(uint8_t ch, uint32_t azi) const {
    unsigned int i = std::floor(1.f * azi / STEP3);
    unsigned int l = azi - i * STEP3;
    float k = 1.f * l / STEP3;
    return round((1 - k) * elevation_offset[ch * CORRECTION_AZIMUTH_NUM + i] +
                 k * elevation_offset[ch * CORRECTION_AZIMUTH_NUM + i + 1]);
  }
};

class Udp4_3_Parser : public GeneralParser {
 public:
  Udp4_3_Parser();
  virtual ~Udp4_3_Parser();

  virtual dwStatus GetDecoderConstants(_dwSensorLidarDecoder_constants* constants) override;
  
  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* data, dwLidarPointRTHI* pointRTHI) override;     
  
  // Get vectical angle of each channel from PandarATCorrections
  int16_t GetVecticalAngle(int channel) override;

private:
  int ParseCorrectionString(char *correction_string) override;
  // Save correction file of azimuth and elevation
  PandarATCorrections m_PandarAT_corrections;
};

#endif  // UDP4_3_PARSER_H_
