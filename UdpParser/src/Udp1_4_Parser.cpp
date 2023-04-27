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

#include <iostream>
#include "Udp1_4_Parser.h"

Udp1_4_Parser::Udp1_4_Parser() {}

Udp1_4_Parser::~Udp1_4_Parser() { 
  // printf("release general parser\n"); 
}

dwStatus Udp1_4_Parser::GetDecoderConstants(_dwSensorLidarDecoder_constants* constants) {
    // printf("GetDecoderConstants: \n");
    // Each packet contains QT的一个包里1127个byte, 按1500, 雷达是单回波还是双回波，频率10Hz已确定 15000 150000 15000
    constants->maxPayloadSize = 1500;
    // 每秒包数量，360/0.4*10 = 9000，如果是双回波的多出一倍*2  18000 36000 
    // !std::bad_alloc happens if value is too small, narrow memory 900000 450000 ok, but 90000 fails? 
    constants->properties.packetsPerSecond = 9000;
    // !Influence the display of point cloud, 9000 * 128 * 2 = 2304000
    constants->properties.pointsPerSecond = 23040000;
    // 10Hz 10 circle per second
    constants->properties.spinFrequency = 10;
    // 900 9000 90000
    constants->properties.packetsPerSpin = 900;
    // 256 
    constants->properties.pointsPerPacket = 256;
    
    constants->properties.pointsPerSpin = 230400;
    constants->properties.pointStride = 8;
    constants->properties.horizontalFOVStart = deg2Rad(0);
    constants->properties.horizontalFOVEnd = deg2Rad(360);
    // QT有可能40 128 64， 从UDP包获得，以为128线，仅40， 参数传入尽量对
    constants->properties.numberOfRows = m_nLaserNum;
    // 说明书上??，实际校正文件中的有差别， 角度文件第一个，和最后一个
    constants->properties.verticalFOVStart = deg2Rad(-14);
    constants->properties.verticalFOVEnd = deg2Rad(26);
    // 我们的只有128线，256这个数组大小
    for (int i = 0; i < m_nLaserNum; i++) {
        if(m_bGetCorrectionFile == true && m_vEleCorrection.empty() == false) {
            constants->properties.verticalAngles[i] = deg2Rad(m_vEleCorrection[i] / m_iAziCorrUnit);
            // printf("verticalAngles: %f \n", constants->properties.verticalAngles[i]);
        }
    }

    // printLidarProperty(&constants->properties);
    return DW_SUCCESS;
}

dwStatus Udp1_4_Parser::ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                  dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) {
  // return DW_FAILURE;
  if (buffer[0] != 0xEE || buffer[1] != 0xFF || length < 0) {
    printf("Udp1_4_Parser: ParserOnePacket, invalid packet %x %x\n", buffer[0], buffer[1]);
    return DW_FAILURE;
  }
  const HS_LIDAR_HEADER_ME_V4 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_ME_V4 *>(
          &(buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));
  // pHeader->Print();
  // point to azimuth of udp start block
  const HS_LIDAR_BODY_AZIMUTH_ME_V4 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4));
  int32_t azimuth = pAzimuth->GetAzimuth();
  m_nBlockNum = pHeader->GetBlockNum();
  m_nLaserNum = pHeader->GetLaserNum();
  // pAzimuth->Print();
  
  const auto *pTail = reinterpret_cast<const HS_LIDAR_TAIL_ME_V4 *>(
      (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4) +
      GetDataBodySize(pHeader) + sizeof(HS_LIDAR_BODY_CRC_ME_V4) + 
      (pHeader->HasFuncSafety() ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4) : 0));
  // pTail->Print();
  m_u16SpinSpeed = pTail->m_u16MotorSpeed;
  m_bIsDualReturn = false;
  output->duration = pTail->GetTimestamp() - 100000;
  output->hostTimestamp = 0;
  output->maxPoints = m_nBlockNum * m_nLaserNum;
  // TODO from the correction file
  if (m_vEleCorrection.empty() == true) {
    printf("ParserOnePacket: no calibration string loaded Error \n");
    return DW_FAILURE;
  }
  output->maxVerticalAngleRad = m_vEleCorrection[m_nLaserNum - 1] / 1000 / 180 * M_PI;
  output->minVerticalAngleRad = m_vEleCorrection[0] / 1000 / 180 * M_PI;
  output->nPoints = m_nBlockNum * m_nLaserNum;
  // TODO scanComplete必须要有true状态，否则崩
  output->scanComplete = false;
  // Error, not the same
  // output->sensorTimestamp = GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());
  output->sensorTimestamp = this->GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());

  int index = 0;
  float minAzimuth = -361;
  float maxAzimuth = 361;
  for (int blockID = 0; blockID < m_nBlockNum; blockID++) {
    // point to channel unit addr
    if (pHeader->HasConfidenceLevel()) {
      printf("Not supported! HasConfidenceLevel");
    } else {
      const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 *pChnUnitNoConf =
          reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 *>(
              (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4));
      // pAzimuth->Print();
      azimuth = pAzimuth->GetAzimuth();
      // point to next block azimuth addr
      pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4) * m_nLaserNum);
      for (int laserID = 0; laserID < m_nLaserNum; laserID++) {
        int32_t elevation = this->m_vEleCorrection[laserID];
        elevation = (360000 + elevation) % 360000;  //TODO No need
        int32_t aziCorr = this->CalibrateAzimuth(azimuth, laserID);

        double distance = static_cast<double>(pChnUnitNoConf->GetDistance()) * pHeader->GetDistUnit();
        uint8_t intensity = pChnUnitNoConf->GetReflectivity();
        this->ComputeDwPoint(pointXYZI[index], pointRTHI[index], distance, elevation, aziCorr, intensity);
        // PrintDwPoint(&pointXYZI[index]);
        ++ index;
        pChnUnitNoConf = pChnUnitNoConf + 1;
        // pChnUnitNoConf->Print();
      }  // iterate laserId

      if (IsNeedFrameSplit(azimuth)) {
        output->scanComplete = true;
      }
      this->m_u16LastAzimuth = azimuth;
      if (blockID == 0) minAzimuth = azimuth;
      else maxAzimuth = azimuth;

    } // noconf situation
  }  // iterate block
  
  output->maxHorizontalAngleRad = this->deg2Rad(maxAzimuth / m_nAziUnitUDP);
  output->minHorizontalAngleRad = this->deg2Rad(minAzimuth / m_nAziUnitUDP);
  output->pointsRTHI = pointRTHI;
  output->pointsXYZI = pointXYZI;
  // PrintDwPoint(&pointXYZI[index-2]);

  return DW_SUCCESS;
}

int16_t Udp1_4_Parser::GetVecticalAngle(int channel) {
  if (channel < 0 || channel >= HS_LIDAR_P128_LASER_NUM) {
    printf("GetVecticalAngle: channel id not in range 0-%d \n", HS_LIDAR_P128_LASER_NUM-1);
    return -1;
  }
  return m_vEleCorrection[channel];
}

unsigned long Udp1_4_Parser::GetDataBodySize(const HS_LIDAR_HEADER_ME_V4 *pHeader) {
  unsigned long bodySize = (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
       (pHeader->HasConfidenceLevel() ? sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4)
                                      : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4)) *
        pHeader->GetLaserNum()) * pHeader->GetBlockNum();
  return bodySize;
}