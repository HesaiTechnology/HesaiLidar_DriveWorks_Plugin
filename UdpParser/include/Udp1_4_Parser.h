/////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (2022) [Hesai Technology] All rights reserved.
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

#ifndef UDP1_4_PARSER_H_
#define UDP1_4_PARSER_H_

// Unit of azimuth in UDP packet 1/100
#define HS_LIDAR_P128_AZIMUTH_UNIT_UDP (100)
#define HS_LIDAR_P128_LASER_NUM (128)

#include "GeneralParser.h"
#include "PointCloudType.h"
#include "HsLidarMeV4.h"

// For Pandar128
class Udp1_4_Parser : public GeneralParser {
 public:
  Udp1_4_Parser();
  virtual ~Udp1_4_Parser();

  dwStatus getDecoderConstants(_dwSensorLidarDecoder_constants* constants) override;
  
  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) override;

  int16_t GetVecticalAngle(int channel) override;

 private:
  // to be updated by the UDP packet
  int m_nLaserNum = HS_LIDAR_P128_LASER_NUM;
  // block number in a UDP packet
  int m_nBlockNum = 2;
  // unit of azimth angle from UDP packet
  const int m_nAziUnitUDP = HS_LIDAR_P128_AZIMUTH_UNIT_UDP;

  unsigned long GetDataBodySize(const HS_LIDAR_HEADER_ME_V4 *pHeader);
};

#endif  // UDP1_4_PARSER_H_
