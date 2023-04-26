#include <iostream>
#include "Udp1_4_Parser.h"
#include "HsLidarMeV4.h"

Udp1_4_Parser::Udp1_4_Parser() {
  m_iMotorSpeed = 0;
  m_iReturnMode = 0;
}

Udp1_4_Parser::~Udp1_4_Parser() { printf("release general parser\n"); }

dwStatus Udp1_4_Parser::getDecoderConstants(_dwSensorLidarDecoder_constants* constants) {
    // printf("getDecoderConstants: \n");
    // QT的一个包里1127个byte, 按1500, 雷达是单回波还是双回波，频率10Hz已确定 15000 150000 15000
    constants->maxPayloadSize = 1500;
    // 每秒包数量，360/0.4*10 = 9000，如果是双回波的多出一倍*2  18000 36000 
    // 发生std::bad_alloc 900000 450000 90000不行了， 原因怀疑是内存分配不够？
    constants->properties.packetsPerSecond = 9000;
    // 每秒点数量，9000 * 128 * 2 = 2304000, 该参数直接影响实时点云显示！
    constants->properties.pointsPerSecond = 23040000;
    // 10Hz 每秒转10圈
    constants->properties.spinFrequency = 10;
    // 900 9000 90000
    constants->properties.packetsPerSpin = 900;
    // 256 
    constants->properties.pointsPerPacket = 2560;
    
    constants->properties.pointsPerSpin = 230400;
    constants->properties.pointStride = 8;
    // 支持关调一些激光器，动态检查FOV， 如果用户关了一些FOV
    constants->properties.horizontalFOVStart = deg2Rad(0);
    constants->properties.horizontalFOVEnd = deg2Rad(360);
    // QT有可能40 128 64， 从UDP包获得，以为128线，仅40， 参数传入尽量对
    constants->properties.numberOfRows = 128;
    // 说明书上??，实际校正文件中的有差别， 角度文件第一个，和最后一个
    constants->properties.verticalFOVStart = deg2Rad(-14);
    constants->properties.verticalFOVEnd = deg2Rad(26);
    // 我们的只有128线，256这个数组大小
    for (int i = 0; i < 128; i++) {
        if(m_bGetCorrectionFile == true && m_vEleCorrection.empty() == false) {
            constants->properties.verticalAngles[i] = deg2Rad(m_vEleCorrection[i] / HS_LIDAR_P128_AZIMUTH_UNIT);
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
  // pAzimuth->Print();
  const auto *pTail = reinterpret_cast<const HS_LIDAR_TAIL_ME_V4 *>(
      (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4) +
      (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
       (pHeader->HasConfidenceLevel()
            ? sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4)
            : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4)) *
           pHeader->GetLaserNum()) *
          pHeader->GetBlockNum() +
      sizeof(HS_LIDAR_BODY_CRC_ME_V4) +
      (pHeader->HasFuncSafety() ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4) : 0));
  // pTail->Print();
  // return DW_SUCCESS;
  m_u16SpinSpeed = pTail->m_u16MotorSpeed;
  m_bIsDualReturn = false;
  output->duration = pTail->GetTimestamp() - 100000;
  output->hostTimestamp = 0;
  output->maxPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // TODO from the correction file
  if (m_vEleCorrection.empty() == true) {
    printf("ParserOnePacket: no calibration string loaded Error \n");
    return DW_FAILURE;
  }
  output->maxVerticalAngleRad = m_vEleCorrection[pHeader->GetLaserNum() - 1] / 1000 / 180 * M_PI;
  output->minVerticalAngleRad = m_vEleCorrection[0] / 1000 / 180 * M_PI;
  output->nPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // TODO scanComplete必须要有true状态，否则崩
  output->scanComplete = false;
  // output->sensorTimestamp = GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());
  output->sensorTimestamp = GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());

  int index = 0;
  float minAzimuth = -361;
  float maxAzimuth = 361;
  for (int i = 0; i < pHeader->GetBlockNum(); i++) {
    // point to channel unit addr
    if (pHeader->HasConfidenceLevel()) {
      printf("NNOOOONNNNONON HasConfidenceLevel");
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
          sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4) * pHeader->GetLaserNum());
      // 1 00     360 00
      for (unsigned int j = 0; j < pHeader->GetLaserNum(); j++) {
        int32_t elevation = this->m_vEleCorrection[j];
        elevation = (360000 + elevation) % 360000;  //TODO No need
        int32_t aziCorr = this->CalibrateAzimuth(azimuth, j);

        double distance = static_cast<double>(pChnUnitNoConf->GetDistance()) * pHeader->GetDistUnit();
        uint8_t intensity = pChnUnitNoConf->GetReflectivity();
        this->ComputeDwPoint(pointXYZI[index], pointRTHI[index], distance, elevation, aziCorr, intensity);
        // PrintDwPoint(&pointXYZI[index]);
        ++ index;
        // pChnUnitNoConf->Print();  // failed! no correct
        pChnUnitNoConf = pChnUnitNoConf + 1;
      }  // iterate laserId

      if (IsNeedFrameSplit(azimuth)) {
        output->scanComplete = true;
      }
      this->m_u16LastAzimuth = azimuth;
      if (i == 0) minAzimuth = azimuth;
      else maxAzimuth = azimuth;

    } // noconf situation
  }  // iterate block
  
  output->maxHorizontalAngleRad = ((maxAzimuth) / 100.0f) / 180 * M_PI;
  output->minHorizontalAngleRad = ((minAzimuth) / 100.0f) / 180 * M_PI;
  output->pointsRTHI = pointRTHI;
  output->pointsXYZI = pointXYZI;
  // really == 8 and always
  // printf("lens = %ld \n", sizeof(output->pointsXYZI));
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