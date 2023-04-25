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
    constants->properties.pointsPerSecond = 2304000;
    // 10Hz 每秒转10圈
    constants->properties.spinFrequency = 10;

    constants->properties.packetsPerSpin = 900;
    constants->properties.pointsPerPacket = 256;
    
    constants->properties.pointsPerSpin = 230400;
    constants->properties.pointStride = 8;
    // 支持关调一些激光器，动态检查FOV， 如果用户关了一些FOV
    constants->properties.horizontalFOVStart = deg2Rad(0);
    constants->properties.horizontalFOVEnd = deg2Rad(360);
    // QT有可能40 128 64， 从UDP包获得，以为128线，仅40， 参数传入尽量对
    constants->properties.numberOfRows = 128;
    // 说明书上-52.6，实际校正文件中的有差别， 角度文件第一个，和最后一个
    constants->properties.verticalFOVStart = deg2Rad(-52.6);
    constants->properties.verticalFOVEnd = deg2Rad(52.6);
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
  if (buffer[0] != 0xEE || buffer[1] != 0xFF || length < 0) {
    printf("Udp1_4_Parser: ParserOnePacket, invalid packet %x %x\n", buffer[0], buffer[1]);
    return DW_FAILURE;
  }
  const HS_LIDAR_HEADER_ME_V4 *pHeader = reinterpret_cast<const HS_LIDAR_HEADER_ME_V4 *>(
                                        &(buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));
  pHeader->Print();
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
  pTail->Print();
  m_u16SpinSpeed = pTail->m_u16MotorSpeed;
  // TODO
  m_bIsDualReturn = false;
  // TODO
  output->duration = pTail->GetTimestamp() - 100000;
  output->hostTimestamp = 0;
  output->maxPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  
  // TODO from the correction file
  output->maxVerticalAngleRad = 0;
  output->minVerticalAngleRad = 0;

  output->nPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // TODO scanComplete必须要有true状态，否则崩
  output->scanComplete = false;
  output->sensorTimestamp = GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());

  // point to azimuth of udp start block
  const HS_LIDAR_BODY_AZIMUTH_ME_V4 *pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ME_V4));

  int index = 0;
  float minAzimuth = -361;
  float maxAzimuth = 361;
  for (int i = 0; i < pHeader->GetBlockNum(); i++) {
    // point to channel unit addr
    void *pChnUnit = nullptr;  // TODO 这样子定义通用的struct指针对吗 struct *pChnUnit
    if (pHeader->HasConfidenceLevel()) {
      pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_ME_V4 *>(
          (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4));
          // point to next block azimuth addr
      pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
        (const unsigned char *)pAzimuth +
        sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
        sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4) * pHeader->GetLaserNum());
    } else {
      pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 *>(
              (const unsigned char *)pAzimuth +
              sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4));
      pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ME_V4 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
          sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4) * pHeader->GetLaserNum());
    }

    uint16_t elevation = 0;
    uint16_t azimuth = pAzimuth->GetAzimuth();
    for (int j = 0; j < pHeader->GetLaserNum(); ++j) {
      // point cloud data
      double distance = pChnUnit->GetDistance() * pHeader->GetDistUnit();
      // TODO Get distance now 需要解析出XYZI 这里的单位还需要调整一下！不对
      if (this->m_bGetCorrectionFile) {
        elevation = this->m_vEleCorrection[i] * HS_LIDAR_P128_AZIMUTH_UNIT;
        elevation = (HS_LIDAR_P128_AZIMUTH_SIZE + elevation) % HS_LIDAR_P128_AZIMUTH_SIZE;
        azimuth = azimuth + this->m_vAziCorrection[i] * HS_LIDAR_P128_AZIMUTH_UNIT;
        azimuth = (HS_LIDAR_P128_AZIMUTH_SIZE + azimuth) % HS_LIDAR_P128_AZIMUTH_SIZE;
      }
      
      double xyDistance = distance * this->m_fCosAllAngle[(elevation)];
      pChnUnit = pChnUnit + 1;
      pointXYZI[index].x = xyDistance * this->m_fSinAllAngle[azimuth];
      pointXYZI[index].y = xyDistance * this->m_fCosAllAngle[azimuth];
      pointXYZI[index].z = distance * this->m_fSinAllAngle[elevation];
      pointXYZI[index].intensity = pChnUnit->GetReflectivity();

      pointRTHI[index].radius = distance;
      pointRTHI[index].theta = azimuth / HS_LIDAR_P128_AZIMUTH_UNIT / 180 * M_PI;
      pointRTHI[index].phi = elevation / HS_LIDAR_P128_AZIMUTH_UNIT / 180 * M_PI;
      pointRTHI[index].intensity = pChnUnit->GetReflectivity();
      ++ index;
    }  // 内循环遍历线束
    if (IsNeedFrameSplit(azimuth)) {
      output.scanComplete = true;
    }
    this->m_u16LastAzimuth = azimuth;
  }
}

int16_t Udp1_4_Parser::GetVecticalAngle(int channel) {
  if (channel < 0 || channel >= HS_LIDAR_P128_LASER_NUM) {
    printf("GetVecticalAngle: channel id not in range 0-%d \n", HS_LIDAR_P128_LASER_NUM-1);
    return -1;
  }
  return m_vEleCorrection[channel];
}