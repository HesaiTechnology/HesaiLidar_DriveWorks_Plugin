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

#include <sstream>
#include "Udp3_2_Parser.h"

Udp3_2_Parser::Udp3_2_Parser() {}

Udp3_2_Parser::~Udp3_2_Parser() { 
  // printf("release Udp3_2_Parser\n"); 
}


dwStatus Udp3_2_Parser::ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                        dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI)
{
  // printf("Udp3_2_Parser:ParserOnePacket, lens=%lu \n", length);
  // printf(" %x yes %x \n", buffer[0], buffer[1]);
  if (length < 0 || buffer[0] != 0xEE || buffer[1] != 0xFF) {
    printf("Udp3_2_Parser: ParserOnePacket, invalid packet %x %x\n", buffer[0], buffer[1]);
    return DW_FAILURE;
  }
  const HS_LIDAR_HEADER_QT_V2 *pHeader = reinterpret_cast<const HS_LIDAR_HEADER_QT_V2 *>(
                                         &(buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));
  // pHeader->Print();
  const HS_LIDAR_TAIL_QT_V2 *pTail = reinterpret_cast<const HS_LIDAR_TAIL_QT_V2 *>(
                          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2) +
                          (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) + sizeof(HS_LIDAR_BODY_CHN_UNIT_QT_V2) * pHeader->GetLaserNum()) *
                          pHeader->GetBlockNum() + sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
                          (pHeader->HasFunctionSafety() ? sizeof(HS_LIDAR_FUNCTION_SAFETY) : 0));
  // pTail->Print();
  m_u16SpinSpeed = pTail->m_u16MotorSpeed;
  // dual return won't affect decoding, just the azimuth of two blocks are the same, more points
  m_bIsDualReturn = pTail->IsDualReturn();
  output->duration = pTail->GetTimestamp() - 100000;
  // from outside this func
  output->hostTimestamp = 0; 
  output->maxPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // !Attention m_vEleCorrection and m_vAziCorrection must be initialized by func LoadCorrectionString
  if (m_vEleCorrection.empty() == true) {
    // printf("Udp3_2_Parser: ParserOnePacket, no calibration string loaded Error \n");
    return DW_FAILURE;
  }
  output->maxVerticalAngleRad = m_vEleCorrection[pHeader->GetLaserNum() - 1] / 1000 / 180 * M_PI;
  output->minVerticalAngleRad = m_vEleCorrection[0] / 1000 / 180 * M_PI;
  // vital parameter to assign memory
  output->nPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // scanComplete must set false, then true when one scan is completed
  output->scanComplete = false;
  // output->sensorTimestamp = GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());
  // output->sensorTimestamp = pTail->GetTimestamp();
  output->sensorTimestamp = this->GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());

  const HS_LIDAR_BODY_AZIMUTH_QT_V2 *pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2));
  // pAzimuth->Print();
  const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *>(
          (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2));
  // pChnUnit->Print();

  // index of the packet, from block count * laser nums, 2*128
  int index = 0;
  float minAzimuth = -361;
  float maxAzimuth = 361;
  unsigned int blocknum = pHeader->GetBlockNum();
  for (unsigned int i = 0; i < blocknum; i++) {
    uint32_t azimuth = pAzimuth->GetAzimuth();
    // azimuth corresponds to a block
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *>(
               (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2));
    // then pAzimuth points to next azimuth
    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
        (const unsigned char *)pAzimuth +
        sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) +
        sizeof(HS_LIDAR_BODY_CHN_UNIT_QT_V2) * pHeader->GetLaserNum());
    int loopIndex = (pTail->GetModeFlag() + (i / ((pTail->GetReturnMode() < 0x39) ? 1 : 2)) + 1) % 2;

    for (unsigned int j = 0; j < pHeader->GetLaserNum(); j++) {
      uint16_t u16Distance = pChnUnit->GetDistance();
      uint8_t u8Intensity = pChnUnit->GetReflectivity();
      // uint8_t u8Confidence = pChnUnit->GetConfidenceLevel();
      double distance = static_cast<double>(u16Distance) * pHeader->GetDistUnit();
      uint32_t azimuthCorr = 0;
      uint32_t elevationCorr = 0;
      pChnUnit = pChnUnit + 1;

      if (m_vEleCorrection.size() >= j && m_vAziCorrection.size() >= j) {
        int laserId = j;
        if (pHeader->HasSelfDefine() && m_PandarQTChannelConfig.m_bIsChannelConfigObtained 
            && i < m_PandarQTChannelConfig.m_vChannelConfigTable[loopIndex].size()) {
            laserId = m_PandarQTChannelConfig.m_vChannelConfigTable[loopIndex][j] - 1;
        }
        elevationCorr = m_vEleCorrection[laserId];
        // azimuth unit from UDP packet is 100, e.g. 1.23 = 123.
        // however, azimuth unit from correction file is 1000, e.g. 1.234 = 1234
        azimuthCorr = azimuth * 10 + m_vAziCorrection[laserId];
        // if (m_bEnableFireTimeCorrection) { ////!! TODO
        //     azimuthCorr += GetFiretimesCorrection(laserId, pTail->GetMotorSpeed(), loopIndex);
        // }
      }
      elevationCorr = (HS_LIDAR_QT128_AZIMUTH_SIZE + elevationCorr) % HS_LIDAR_QT128_AZIMUTH_SIZE;
      azimuthCorr = (HS_LIDAR_QT128_AZIMUTH_SIZE + azimuthCorr) % HS_LIDAR_QT128_AZIMUTH_SIZE;
      // printf("azimuthCorr: %d, elevationCorr: %d \n", azimuthCorr, elevationCorr);
      this->ComputeDwPoint(pointXYZI[index], pointRTHI[index], distance, elevationCorr, azimuthCorr, u8Intensity);
      ++ index;
      // PrintDwPoint(&pointRTHI[index]);
      // PrintDwPoint(&pointXYZI[index]);
    } // cycle laser channel
    
    if (IsNeedFrameSplit(azimuth)) {
      output->scanComplete = true;
    }
    m_u16LastAzimuth = azimuth;
    // As only two block exist, the primary one is minimum
    if (i == 0) minAzimuth = azimuth;
    else maxAzimuth = azimuth;
  } // cycle block
  // PrintDwPoint(&pointXYZI[index-2]);

  output->maxHorizontalAngleRad = ((maxAzimuth) / 100.0f) / 180 * M_PI;
  output->minHorizontalAngleRad = ((minAzimuth) / 100.0f) / 180 * M_PI;
  // !the display only rely on xyzi
  output->pointsRTHI = pointRTHI;
  output->pointsXYZI = pointXYZI;
  // PrintDwPacket(output);

  return DW_SUCCESS;
}

dwStatus Udp3_2_Parser::GetDecoderConstants(_dwSensorLidarDecoder_constants* constants) {
  // printf("GetDecoderConstants: \n");
  // Each packet contains 1127 bytes for QT128, use 1500
  constants->maxPayloadSize = 1500;
  // Packet nums per second，360/0.4*10 = 9000
  // Vital - param to detect a gap in sensor timestamp
  constants->properties.packetsPerSecond = 9000 / (m_bIsDualReturn ? 1 : 2);
  // Vital - affect the display of live sensor, point nums per second: 9000 * 128 * 2 = 2304000
  constants->properties.pointsPerSecond = 2304000 / (m_bIsDualReturn ? 1 : 2);
  constants->properties.spinFrequency = m_u16SpinSpeed / 60.0f;

  // constants->properties.packetsPerSpin = 900;
  // constants->properties.pointsPerSpin = 230400;
  constants->properties.pointsPerPacket = 256;
  constants->properties.pointStride = 8;
  // TODO support qt lidar closes some laser channel, namely dynamic FOV
  constants->properties.horizontalFOVStart = deg2Rad(0);
  constants->properties.horizontalFOVEnd = deg2Rad(360);
  // TODO QT can use customized channels 40 128 64
  constants->properties.numberOfRows = 128;
  // From the correction file, the first and last
  constants->properties.verticalFOVStart = deg2Rad(-52.6);
  constants->properties.verticalFOVEnd = deg2Rad(52.6);
  for (int i = 0; i < 128; i++) {
      if(m_bGetCorrectionFile == true && m_vEleCorrection.empty() == false) {
          constants->properties.verticalAngles[i] = deg2Rad(m_vEleCorrection[i] / HS_LIDAR_QT128_AZIMUTH_UNIT);
          // printf("verticalAngles: %f \n", constants->properties.verticalAngles[i]);
      }
  }

  // printLidarProperty(&constants->properties);
  return DW_SUCCESS;
}

int16_t Udp3_2_Parser::GetVecticalAngle(int channel) {
  if (channel < 0 || channel >= HS_LIDAR_QT128_LASER_NUM) {
    printf("GetVecticalAngle: channel id not in range 0-%d \n", HS_LIDAR_QT128_LASER_NUM-1);
    return -1;
  }

  return m_vEleCorrection[channel];
}

int Udp3_2_Parser::LoadFiretimesString(const char *firetimes) {
  // printf("LoadFiretimesString: %s\n",firetimes);
  std::string firetimes_content_str = firetimes;
  std::istringstream fin(firetimes_content_str);
  std::string line;
  if (std::getline(fin, line)) {  // first line sequence,chn id,firetime/us
    // printf("Parse Lidar firetime now...\n");
  }
  std::vector<std::string> firstLine;
  HSSplit(firstLine, line, ',');
  if (firstLine[0] == "EEFF" || firstLine[0] == "eeff") {
    std::array<std::array<float, HS_LIDAR_QT128_LASER_NUM>,
               HS_LIDAR_QT128_LOOP_NUM>
        firetimes;
    firetimes[0].fill(0);
    firetimes[1].fill(0);
    firetimes[2].fill(0);
    firetimes[3].fill(0);
    std::getline(fin, line);
    std::vector<std::string> loopNumLine;
    HSSplit(loopNumLine, line, ',');
    unsigned int loopNum = atoi(loopNumLine[3].c_str());
    std::getline(fin, line);
    for (int i = 0; i < HS_LIDAR_QT128_LASER_NUM; i++) {
      std::getline(fin, line);
      std::vector<std::string> ChannelLine;
      HSSplit(ChannelLine, line, ',');
      for (unsigned int j = 0; j < loopNum; j++) {
        if (ChannelLine.size() == loopNum * 2) {
          int laserId = atoi(ChannelLine[j * 2].c_str()) - 1;
          firetimes[j][laserId] = std::stof(ChannelLine[j * 2 + 1].c_str());
          // printf("loop num=%d, laserId =%d, firetime = %f\n", j, laserId, firetimes[j][laserId]);
        } else {
          printf("loop num is not equal to the first channel line\n");
          return -1;
        }
      }
    }
    m_vQT128Firetime = firetimes;
  } else {
    printf("firetime file delimiter is wrong\n");
    return -1;
  }
  return 0;
}

void Udp3_2_Parser::LoadFiretimesFile(std::string firetimes_path) {
  std::ifstream fin(firetimes_path);
  if (fin.is_open() == false) {
    printf("LoadFiretimesFile: Open firetimes file Error, path=%s\n", firetimes_path.c_str());
    return;
  }
  int length = 0;
  fin.seekg(0, std::ios::end);
  length = fin.tellg();
  fin.seekg(0, std::ios::beg);
  char *buffer = new char[length];
  fin.read(buffer, length);
  fin.close();
  
  int ret =  LoadFiretimesString(buffer);
  if (ret != 0) {
    printf("LoadFiretimesString: Parse local firetimes file Error");
  }
  delete[] buffer;

}

int Udp3_2_Parser::LoadChannelConfigString(const char *channelconfig) {
  // printf("LoadChannelConfigString: \n");
  // printf("%s\n",channelconfig);
  std::string channel_config_content_str = channelconfig;
  m_PandarQTChannelConfig.m_bIsChannelConfigObtained = false;
  std::istringstream ifs(channel_config_content_str);
  std::string line;

  std::getline(ifs, line);
  std::vector<std::string> versionLine;
  HSSplit(versionLine, line, ',');
  if (versionLine[0] == "EEFF" || versionLine[0] == "eeff") {
    m_PandarQTChannelConfig.m_u8MajorVersion =
        std::stoi(versionLine[1].c_str());
    m_PandarQTChannelConfig.m_u8MinVersion = std::stoi(versionLine[2].c_str());
  } else {
    printf("channel config file delimiter is wrong\n");
    return -1;
  }
  std::getline(ifs, line);
  std::vector<std::string> channelNumLine;
  HSSplit(channelNumLine, line, ',');
  m_PandarQTChannelConfig.m_u8LaserNum = std::stoi(channelNumLine[1].c_str());
  m_PandarQTChannelConfig.m_u8BlockNum = std::stoi(channelNumLine[3].c_str());
  if (m_PandarQTChannelConfig.m_u8LaserNum <= 0 ||
      m_PandarQTChannelConfig.m_u8BlockNum <= 0) {
    printf("LaserNum:%d, BlockNum:%d", m_PandarQTChannelConfig.m_u8LaserNum, m_PandarQTChannelConfig.m_u8BlockNum);
    return -1;
  }
  std::getline(ifs, line);
  std::vector<std::string> firstChannelLine;
  HSSplit(firstChannelLine, line, ',');
  unsigned int loop_num = firstChannelLine.size();
  m_PandarQTChannelConfig.m_vChannelConfigTable.resize(loop_num);

  for (unsigned int i = 0; i < loop_num; i++) {
    m_PandarQTChannelConfig.m_vChannelConfigTable[i].resize(
        m_PandarQTChannelConfig.m_u8LaserNum);
  }
  for (int i = 0; i < m_PandarQTChannelConfig.m_u8LaserNum; i++) {
    std::getline(ifs, line);
    std::vector<std::string> ChannelLine;
    HSSplit(ChannelLine, line, ',');
    for (unsigned int j = 0; j < loop_num; j++) {
      if (ChannelLine.size() == loop_num) {
        m_PandarQTChannelConfig.m_vChannelConfigTable[j][i] =
            std::stoi(ChannelLine[j].c_str());
      } else {
        printf("loop num is not equal to the first channel line\n");
        return -1;
      }
    }
  }
  std::getline(ifs, line);
  m_PandarQTChannelConfig.m_sHashValue = line;
  m_PandarQTChannelConfig.m_bIsChannelConfigObtained = true;

  return 0;
}

void Udp3_2_Parser::LoadChannelConfigFile(std::string channel_config_path) {
  int ret = 0;
  std::ifstream fin(channel_config_path);
  if (fin.is_open()) {
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    ret = LoadChannelConfigString(buffer);
    if (ret != 0) {
      printf("Parse local channel congfig file Error\n");
    }
    delete[] buffer;
  } else {
    printf("Open channel congfig file failed\n");
    return;
  }
}

double Udp3_2_Parser::GetFiretimesCorrection(int laserId, double speed,
                                             int loopIndex) {
  if (m_vQT128Firetime.empty() == true) {
    printf("GetFiretimesCorrection: m_vQT128Firetime is empty, Error\n");
    return -1;
  }
  //需要一个int值， 6e-6， round一下这个结果， 0.1度修正， 100， 0.001
  return m_vQT128Firetime[loopIndex][laserId] * speed * 6E-6;
}
