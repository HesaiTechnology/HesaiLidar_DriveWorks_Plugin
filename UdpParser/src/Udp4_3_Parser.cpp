#include "Udp4_3_Parser.h"

#include "HsLidarStV3.h"
#include "LidarProtocolHeader.h"
#include <fstream>
// #include "UdpParser.h"

Udp4_3_Parser::Udp4_3_Parser() {
  // printf("Udp4_3_Parser: creating parser for lidar AT128 \n");
  m_bGetCorrectionFile = false;
  m_bIsDualReturn = true;
  m_u16SpinSpeed = 2000;
}

Udp4_3_Parser::~Udp4_3_Parser() { 
  // printf("release Udp4_3_Parser\n"); 
}

int Udp4_3_Parser::ParseCorrectionString(char *correction_string) {
  // printf("ParseCorrectionString: parsing calibration file\n");
  try {
    char *p = correction_string;
    PandarATCorrectionsHeader header = *(PandarATCorrectionsHeader *)p;
    if (0xee == header.delimiter[0] && 0xff == header.delimiter[1]) {
      switch (header.version[1]) {
        case 3: {
          m_PandarAT_corrections.header = header;
          auto frame_num = m_PandarAT_corrections.header.frame_number;
          auto channel_num = m_PandarAT_corrections.header.channel_number;
          p += sizeof(PandarATCorrectionsHeader);
          memcpy((void *)&m_PandarAT_corrections.start_frame, p,
                 sizeof(uint16_t) * frame_num);
          p += sizeof(uint16_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.end_frame, p,
                 sizeof(uint16_t) * frame_num);
          p += sizeof(uint16_t) * frame_num;
          // printf("frame_num: %d\n", frame_num);
          // printf("start_frame, end_frame: \n");
          for (int i = 0; i < frame_num; ++i)
            // printf("%lf,   %lf\n",
            //        m_PandarAT_corrections.start_frame[i] / 100.f,
            //        m_PandarAT_corrections.end_frame[i] / 100.f);
          memcpy((void *)&m_PandarAT_corrections.azimuth, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.elevation, p,
                 sizeof(int16_t) * channel_num);
          p += sizeof(int16_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.azimuth_offset, p,
                 sizeof(int8_t) * CIRCLE_ANGLE);
          p += sizeof(int8_t) * CIRCLE_ANGLE;
          memcpy((void *)&m_PandarAT_corrections.elevation_offset, p,
                 sizeof(int8_t) * CIRCLE_ANGLE);
          p += sizeof(int8_t) * CIRCLE_ANGLE;
          memcpy((void *)&m_PandarAT_corrections.SHA256, p,
                 sizeof(uint8_t) * 32);
          p += sizeof(uint8_t) * 32;
          m_bGetCorrectionFile = true;
          return 0;
        } break;
        case 5: {
          m_PandarAT_corrections.header = header;
          auto frame_num = m_PandarAT_corrections.header.frame_number;
          auto channel_num = m_PandarAT_corrections.header.channel_number;
          p += sizeof(PandarATCorrectionsHeader);
          memcpy((void *)&m_PandarAT_corrections.l.start_frame, p,
                 sizeof(uint32_t) * frame_num);
          p += sizeof(uint32_t) * frame_num;
          memcpy((void *)&m_PandarAT_corrections.l.end_frame, p,
                 sizeof(uint32_t) * frame_num);
          p += sizeof(uint32_t) * frame_num;
          // printf("frame_num: %d\n", frame_num);
          // printf("start_frame, end_frame: \n");
          // for (int i = 0; i < frame_num; ++i)
          //   printf("%lf,   %lf\n",
          //          m_PandarAT_corrections.l.start_frame[i] /
          //              (FINE_AZIMUTH_UNIT * 100.f),
          //          m_PandarAT_corrections.l.end_frame[i] /
          //              (FINE_AZIMUTH_UNIT * 100.f));
          memcpy((void *)&m_PandarAT_corrections.l.azimuth, p,
                 sizeof(int32_t) * channel_num);
          p += sizeof(int32_t) * channel_num;
          memcpy((void *)&m_PandarAT_corrections.l.elevation, p,
                 sizeof(int32_t) * channel_num);
          p += sizeof(int32_t) * channel_num;
          auto adjust_length = channel_num * CORRECTION_AZIMUTH_NUM;
          memcpy((void *)&m_PandarAT_corrections.azimuth_offset, p,
                 sizeof(int8_t) * adjust_length);
          p += sizeof(int8_t) * adjust_length;
          memcpy((void *)&m_PandarAT_corrections.elevation_offset, p,
                 sizeof(int8_t) * adjust_length);
          p += sizeof(int8_t) * adjust_length;
          memcpy((void *)&m_PandarAT_corrections.SHA256, p,
                 sizeof(uint8_t) * 32);
          p += sizeof(uint8_t) * 32;
          // printf("frame_num: %d\n", frame_num);
          // printf("start_frame, end_frame: \n");
          for (int i = 0; i < frame_num; ++i) {
            m_PandarAT_corrections.l.start_frame[i] = m_PandarAT_corrections.l.start_frame[i] * m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.l.end_frame[i] = m_PandarAT_corrections.l.end_frame[i] * m_PandarAT_corrections.header.resolution;
            // printf("%lf,   %lf\n", m_PandarAT_corrections.l.start_frame[i] / AZIMUTH_UNIT, m_PandarAT_corrections.l.end_frame[i] / AZIMUTH_UNIT);
          }
          for (int i = 0; i < AT128_LASER_NUM; i++) {
            m_PandarAT_corrections.l.azimuth[i] = m_PandarAT_corrections.l.azimuth[i] * m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.l.elevation[i] = m_PandarAT_corrections.l.elevation[i] * m_PandarAT_corrections.header.resolution;
          }
          for (int i = 0; i < adjust_length; i++) {
            m_PandarAT_corrections.azimuth_offset[i] = m_PandarAT_corrections.azimuth_offset[i] * m_PandarAT_corrections.header.resolution;
            m_PandarAT_corrections.elevation_offset[i] = m_PandarAT_corrections.elevation_offset[i] * m_PandarAT_corrections.header.resolution;
          }

          m_bGetCorrectionFile = true;
          return 0;
        } break;
        default:
          break;
      }
    }

    return -1;
  } catch (const std::exception &e) {
    return -1;
  }

  return -1;
}

dwStatus Udp4_3_Parser::getDecoderConstants(_dwSensorLidarDecoder_constants* constants) {
  // 必传-异常退出, 填10太小直接崩溃 500会崩，800还可以？ 1118AT一个包里byte数
  constants->maxPayloadSize = 1500;
  // 必传-virtual会进入loading黑屏卡死, 过大多加2个0，显示界面错位！加4个0则直接退出 12500
  // Fault parameter to avoid dw printing packet dropping
  constants->properties.packetsPerSecond = 12 / (m_bIsDualReturn ? 1 : 2);
  // 无影响
  constants->properties.packetsPerSpin = 1250 * m_u16SpinSpeed / 2000.0f / (m_bIsDualReturn ? 1 : 2);
  constants->properties.pointsPerPacket = 256;
  // 重要，异常退出，过大加2个0界面小卡一下
  constants->properties.pointsPerSecond = 3200000 / (m_bIsDualReturn ? 1 : 2);
  // 无影响
  constants->properties.pointsPerSpin = 320000 * m_u16SpinSpeed / 2000.0f / (m_bIsDualReturn ? 1 : 2);
  constants->properties.pointStride = 4;
  // 必传-不传界面空的
  constants->properties.spinFrequency = m_u16SpinSpeed / 200.0f;
  // 无影响-垂直水平FOV
  constants->properties.horizontalFOVEnd = 2.8;
  constants->properties.horizontalFOVStart = 0.5;
  constants->properties.numberOfRows = 128;
  constants->properties.verticalFOVEnd = 0.224673;
  constants->properties.verticalFOVStart = -0.216697;
  // 不传无影响
  // printLidarProperty(&constants->properties);
  for (int i = 0; i < 128; i++) {
      if(m_bGetCorrectionFile)
          constants->properties.verticalAngles[i] = deg2Rad(GetVecticalAngle(i) / 25600.0f);
  }

  return DW_SUCCESS;
}

dwStatus Udp4_3_Parser::ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI){
  if (buffer[0] != 0xEE || buffer[1] != 0xFF || length < 0) {
    // printf("Udp4_3_Parser: ParserOnePacket, invalid packet %x %x\n", buffer[0], buffer[1]);
    return DW_FAILURE;
  }
  const HS_LIDAR_HEADER_ST_V3 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_ST_V3 *>(
          &(buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

  const HS_LIDAR_TAIL_ST_V3 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_ST_V3 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V3) +
          (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
           sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
           sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V3) * pHeader->GetLaserNum()) *
              pHeader->GetBlockNum() +
          sizeof(HS_LIDAR_BODY_CRC_ST_V3));
  m_u16SpinSpeed = pTail->m_i16MotorSpeed;
  m_bIsDualReturn = pTail->IsDualReturn();
  // 无用参数
  output->duration = pTail->GetMicroLidarTimeU64() - 100000; 
  // 在函数外已赋值，置为0也没用
  output->hostTimestamp = 0;
  // 如下三条：max min这样的参数一点用都没有
  output->maxPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  output->maxVerticalAngleRad = m_PandarAT_corrections.l.elevation[pHeader->GetLaserNum() - 1] /180 * M_PI/ 25600.0f;
  output->minVerticalAngleRad = m_PandarAT_corrections.l.elevation[0] /180 * M_PI/ 25600.0f;
  // 不填直接崩调，=0界面一个点也没有
  output->nPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // 不填则仅显示很小一部分点云
  output->scanComplete = false;
  // 不填可以播放，只是显示的时间戳不对
  output->sensorTimestamp = pTail->GetMicroLidarTimeU64();
  int index = 0;
  float minAzimuth = 0;
  float maxAzimuth = 0;
  
  const HS_LIDAR_BODY_AZIMUTH_ST_V3 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ST_V3 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_ST_V3));
  const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3 *pFineAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3));

  const HS_LIDAR_BODY_CHN_NNIT_ST_V3 *pChnUnit =
      reinterpret_cast<const HS_LIDAR_BODY_CHN_NNIT_ST_V3 *>(
          (const unsigned char *)pAzimuth +
          sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
          sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3));
  for (int blockid = 0; blockid < pHeader->GetBlockNum(); blockid++) {
    uint16_t u16Azimuth = pAzimuth->GetAzimuth();
    uint8_t u8FineAzimuth = pFineAzimuth->GetFineAzimuth();
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_NNIT_ST_V3 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
        sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3));

    // point to next block azimuth addr
    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_ST_V3 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
        sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
        sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V3) * pHeader->GetLaserNum());
    // point to next block fine azimuth addr
    pFineAzimuth = reinterpret_cast<const HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3 *>(
        (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3));

    int Azimuth = u16Azimuth * FINE_AZIMUTH_UNIT + u8FineAzimuth;
    int count = 0, field = 0;
    if ( m_bGetCorrectionFile) {
      while (count < m_PandarAT_corrections.header.frame_number &&
             (((Azimuth + MAX_AZI_LEN - m_PandarAT_corrections.l.start_frame[field]) % MAX_AZI_LEN +
             (m_PandarAT_corrections.l.end_frame[field] + MAX_AZI_LEN - Azimuth) % MAX_AZI_LEN) !=
             (m_PandarAT_corrections.l.end_frame[field] + MAX_AZI_LEN -
             m_PandarAT_corrections.l.start_frame[field]) % MAX_AZI_LEN)) {
        field = (field + 1) % m_PandarAT_corrections.header.frame_number;
        count++;
      }
      if (count >= m_PandarAT_corrections.header.frame_number) continue;
    }
    auto elevation =0;
    auto azimuth = Azimuth;
    for (int i = 0; i < pHeader->GetLaserNum(); i++) {
      /* for all the units in a block */
      uint16_t u16Distance = pChnUnit->GetDistance();
      uint8_t u8Intensity = pChnUnit->GetReflectivity();
      // uint8_t u8Confidence = pChnUnit->GetConfidenceLevel();
      float distance = static_cast<float>(u16Distance) * pHeader->GetDistUnit();
      pChnUnit = pChnUnit + 1;
      
      if (m_bGetCorrectionFile) {
        elevation = (m_PandarAT_corrections.l.elevation[i] +
                   m_PandarAT_corrections.getElevationAdjustV3(i, Azimuth) *
                       FINE_AZIMUTH_UNIT );
        elevation = (MAX_AZI_LEN + elevation) % MAX_AZI_LEN;
        azimuth = ((Azimuth + MAX_AZI_LEN - m_PandarAT_corrections.l.start_frame[field]) * 2 -
                         m_PandarAT_corrections.l.azimuth[i] +
                         m_PandarAT_corrections.getAzimuthAdjustV3(i, Azimuth) * FINE_AZIMUTH_UNIT);
        azimuth = (MAX_AZI_LEN + azimuth) % MAX_AZI_LEN;
      }      
      float xyDistance = distance * m_PandarAT_corrections.cos_map[(elevation)];

      pointXYZI[index].x = xyDistance * m_PandarAT_corrections.sin_map[(azimuth)];
      pointXYZI[index].y = xyDistance * m_PandarAT_corrections.cos_map[(azimuth)];
      pointXYZI[index].z = distance * m_PandarAT_corrections.sin_map[(elevation)];
      pointXYZI[index].intensity = u8Intensity;
      pointRTHI[index].radius = distance;
      pointRTHI[index].theta = azimuth / AZIMUTH_UNIT / 180 * M_PI;
      pointRTHI[index].phi = elevation / AZIMUTH_UNIT / 180 * M_PI;
      pointRTHI[index].intensity = u8Intensity; 
      index++;
      // PrintDwPoint(&pointXYZI[index]);
    }
    if (IsNeedFrameSplit(u16Azimuth, field)) {
      // 未置true会一直显示loading，几秒后超时退出崩溃
      output->scanComplete = true;
    }
    m_u16LastAzimuth = u16Azimuth;
    if(blockid  == 0 ) minAzimuth =  azimuth;
    else maxAzimuth = azimuth;
    
  }

  // 不传不影响显示
  output->maxHorizontalAngleRad = (maxAzimuth / 25600.0f) / 180 * M_PI;
  output->minHorizontalAngleRad = (minAzimuth / 25600.0f) / 180 * M_PI;
  // 不传点进去只是一个点也不显示，不会崩溃
  output->pointsRTHI = pointRTHI;
  output->pointsXYZI = pointXYZI;

  return DW_SUCCESS;
}

int16_t Udp4_3_Parser::GetVecticalAngle(int channel) {
  if (m_bGetCorrectionFile == false) {
    printf ("GetVecticalAngle: no correction file get, Error");
    return -1;
  }

  return m_PandarAT_corrections.elevation[channel];
}





