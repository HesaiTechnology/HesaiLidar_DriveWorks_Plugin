#include <sstream>
#include "Udp3_2_Parser.h"
// #include <thread>
// #include <chrono>

Udp3_2_Parser::Udp3_2_Parser() {
  // printf("Udp3_2_Parser: creating \n");
  m_iMotorSpeed = 0;
  m_iReturnMode = 0;

  for (int i = 0; i < HS_LIDAR_QT128_AZIMUTH_SIZE; ++i) {
      // 360度对应 2* M_PI, 角度细分0.001 = 1/1000
      sin_map[i] = std::sin(i * 2 * M_PI / HS_LIDAR_QT128_AZIMUTH_SIZE);
      cos_map[i] = std::cos(i * 2 * M_PI / HS_LIDAR_QT128_AZIMUTH_SIZE);
  }
}

Udp3_2_Parser::~Udp3_2_Parser() { 
  // printf("release Udp3_2_Parser\n"); 
}


dwStatus Udp3_2_Parser::ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                        dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI)
{
  // printf("Udp3_2_Parser:ParserOnePacket, lens=%lu \n", length);
  // printf(" %x yes %x \n", buffer[0], buffer[1]);
  // std::chrono::milliseconds dura(2000);
  // std::this_thread::sleep_for(dura);
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
  // 该参数，双回波QT128不存在？
  m_bIsDualReturn = false;
  // 从包头或者包尾获取
  output->duration = pTail->GetTimestamp() - 100000;
  // 时间戳在调完此函数后外部输入
  output->hostTimestamp = 0; 
  output->maxPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // 参数传入不影响，一个包中两block均为同一水平角下数据，垂直方位角相同, 1 234 = /1000 = 1.234角度
  if (m_vEleCorrection.empty() == true) {
    printf("ParserOnePacket: no calibration string loaded Error \n");
    return DW_FAILURE;
  }
  output->maxVerticalAngleRad = m_vEleCorrection[pHeader->GetLaserNum() - 1] / 1000 / 180 * M_PI;
  output->minVerticalAngleRad = m_vEleCorrection[0] / 1000 / 180 * M_PI;
  // 这三个参数引入后，空跑的会断开！！那么几秒？？？
  output->nPoints = pHeader->GetBlockNum() * pHeader->GetLaserNum();
  // scanComplete必须要有true状态，否则崩
  output->scanComplete = false;
  output->sensorTimestamp = GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());

  const HS_LIDAR_BODY_AZIMUTH_QT_V2 *pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_QT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_QT_V2));
  // pAzimuth->Print();
  const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *>(
          (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2));
  // pChnUnit->Print();

  // 包的序号，跨越二重循环，一维包序号保存
  int index = 0;
  float minAzimuth = -361;
  float maxAzimuth = 361;
  // 二维数组循环，两个块，每个块里有128线，从每一线中获取多点数据
  unsigned int blocknum = pHeader->GetBlockNum();
  for (unsigned int i = 0; i < blocknum; i++) {
    uint32_t azimuth = pAzimuth->GetAzimuth();
    // 一个方位角对应一个数据块
    pChnUnit = reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_QT_V2 *>(
               (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2));
    // pAzimuth指针指向块的下一个方位角
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

      // ！m_vEleCorrection&m_vAziCorrection只有在LoadCorrectionString使用后才初始化
      if (m_vEleCorrection.size() >= j && m_vAziCorrection.size() >= j) {
        int laserId = j;
        if (pHeader->HasSelfDefine() && m_PandarQTChannelConfig.m_bIsChannelConfigObtained 
            && i < m_PandarQTChannelConfig.m_vChannelConfigTable[loopIndex].size()) {
            laserId = m_PandarQTChannelConfig.m_vChannelConfigTable[loopIndex][j] - 1;
        }
        elevationCorr = m_vEleCorrection[laserId];
        // UDP包中角度仅两位小数 1 23 = 123度，统一到与角度文件三位小数点 123 0度，故*10
        azimuthCorr = azimuth * 10 + m_vAziCorrection[laserId];
        // if (m_bEnableFireTimeCorrection) { ////!!
        //     azimuthCorr += GetFiretimesCorrection(laserId, pTail->GetMotorSpeed(), loopIndex);
        // }
      }
      elevationCorr = (HS_LIDAR_QT128_AZIMUTH_SIZE + elevationCorr) % HS_LIDAR_QT128_AZIMUTH_SIZE;
      azimuthCorr = (HS_LIDAR_QT128_AZIMUTH_SIZE + azimuthCorr) % HS_LIDAR_QT128_AZIMUTH_SIZE;
      // printf("azimuthCorr: %d, elevationCorr: %d \n", azimuthCorr, elevationCorr);
      float xyDistance = distance * cos_map[elevationCorr];
      pointXYZI[index].x = xyDistance * sin_map[azimuthCorr];
      pointXYZI[index].y = xyDistance * cos_map[azimuthCorr];
      pointXYZI[index].z = distance * sin_map[elevationCorr];
      pointXYZI[index].intensity = static_cast<float>(u8Intensity / 255.0f);  // float type 0-1

      pointRTHI[index].radius = distance;
      pointRTHI[index].theta = azimuthCorr / 1000 / 180 * M_PI;
      pointRTHI[index].phi = elevationCorr / 1000 / 180 * M_PI;
      pointRTHI[index].intensity = static_cast<float>(u8Intensity / 255.0f);
      ++ index;
      // PrintDwPoint(&pointRTHI[index]);
      // PrintDwPoint(&pointXYZI[index]);
    } // 内循环
    
    if (IsNeedFrameSplit(azimuth)) {
      output->scanComplete = true;
    }
    m_u16LastAzimuth = azimuth;
    if (i == 0) minAzimuth = azimuth;
    else maxAzimuth = azimuth;
  } // 外循环
  PrintDwPoint(&pointXYZI[index-2]);
  // auto diff = sizeof(pointXYZI)/sizeof(pointXYZI[0]);
  // printf("shuliang =%d", index);

  // 因为只有两个block，下标0的那个block是首个，取下标最末的那个block
  output->maxHorizontalAngleRad = ((maxAzimuth) / 100.0f) / 180 * M_PI;
  output->minHorizontalAngleRad = ((minAzimuth) / 100.0f) / 180 * M_PI;
  // !the display only rely on xyzi
  output->pointsRTHI = pointRTHI;
  output->pointsXYZI = pointXYZI;
  // PrintDwPacket(output);

  return DW_SUCCESS;
}

dwStatus Udp3_2_Parser::getDecoderConstants(_dwSensorLidarDecoder_constants* constants) {
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
