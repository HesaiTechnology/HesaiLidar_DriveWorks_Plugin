#include "GeneralParser.h"

const std::string GeneralParser::kLidarIPAddr("192.168.1.201");

GeneralParser::GeneralParser() {
  m_iMotorSpeed = 0;
  m_iReturnMode = 0;

  m_u16FramesSize = kFramesNumMax;
  m_u16FrameStartAzimuth = kFrameStartAzimuth;
  m_u16FrameEndAzimuth = kFrameEndAzimuth;
  m_u16LastAzimuth = 0;
  m_iMaxPackageNum = kPackageNumMax;
  // TODO 总感觉这里有问题
  for(int i = 0; i < CIRCLE; ++i) {
      m_fSinAllAngle[i] = std::sin(i * 2 * M_PI / CIRCLE);
      m_fCosAllAngle[i] = std::cos(i * 2 * M_PI / CIRCLE);
  }
}

GeneralParser::~GeneralParser() {
  // printf("release general Parser\n");
}

int GeneralParser::LoadCorrectionFile(std::string correction_path) {
  // printf("GeneralParser::LoadCorrectionFile, path=%s \n", correction_path.c_str());
  std::ifstream fin(correction_path);
  if (fin.is_open() == false) {
    printf("Open correction file Error, path=%s\n", correction_path.c_str());
    return -1;
  }
  int length = 0;
  fin.seekg(0, std::ios::end);
  length = fin.tellg();
  fin.seekg(0, std::ios::beg);
  char *buffer = new char[length];
  fin.read(buffer, length);
  fin.close();

  int ret = ParseCorrectionString(buffer);
  delete[] buffer;
  if (ret != 0) {
    printf("Parse local correction file Error\n");
  } 

  return ret;
}

int GeneralParser::ParseCorrectionString(char* correction_content) {
  // printf("ParseCorrectionString: parsing calibration content\n");
  // printf("%s \n", correction_content);
  std::string correction_content_str = correction_content;
	std::istringstream ifs(correction_content_str);
	std::string line;

	// skip first line "Laser id,Elevation,Azimuth" or "eeff"
  std::getline(ifs, line);

	float elevationList[MAX_LASER_NUM], azimuthList[MAX_LASER_NUM];
	std::vector<std::string>  vfirstLine;
  HSSplit(vfirstLine, line, ',');
	// HSSplit(vfirstLine, line, ',');
	if(vfirstLine[0] == "EEFF" || vfirstLine[0] == "eeff"){
		// skip second line
    std::getline(ifs, line);
	}
    
  int lineCount = 0;
  while (std::getline(ifs, line)) {
    std::vector<std::string>  vLineSplit;
    HSSplit(vLineSplit, line, ',');
    if (vLineSplit.size() < 3) { // skip error line or hash value line 
        continue;
    } else {
        lineCount++;
    }
    float elevation, azimuth;
    int laserId = 0;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> laserId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elevation;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (laserId != lineCount || laserId >= MAX_LASER_NUM) {
        printf("ParseCorrectionString: laser id Error. laser Id=%d, line=%d", laserId, lineCount);
        return -1;
    }
    elevationList[laserId - 1] = elevation;
    azimuthList[laserId - 1] = azimuth;
  }
  m_vEleCorrection.resize(lineCount);
  m_vAziCorrection.resize(lineCount);

	for (int i = 0; i < lineCount; ++i) {
		m_vEleCorrection[i] = static_cast<int32_t> (round(elevationList[i] * 1000));
		m_vAziCorrection[i] = static_cast<int32_t> (round(azimuthList[i] * 1000));
    // printf("m_vEleCorrection, %d \n", m_vEleCorrection[i]);
	}

  m_bGetCorrectionFile = true;
	return 0;
}

int GeneralParser::LoadFiretimesString(const char *firetimes) {
  printf("GeneralParser::LoadFiretimesString, no load\n");
  (void) firetimes;

  return -1;
}

void GeneralParser::LoadFiretimesFile(std::string firetimes_path) {
  printf("GeneralParser::LoadFiretimesFile, no load\n");
  (void) firetimes_path;

  return;
}

int GeneralParser::LoadChannelConfigString(const char *channelconfig) {
  printf("GeneralParser::LoadChannelConfigString, no load\n");
  (void) channelconfig;

  return -1;
}

void GeneralParser::LoadChannelConfigFile(std::string channel_config_path) {
  printf("GeneralParser::LoadChannelConfigFile, no load\n");
  (void) channel_config_path;

  return;
}

void GeneralParser::SetEnableFireTimeCorrection(bool enable) {
    m_bEnableFireTimeCorrection = enable;
}

void GeneralParser::SetEnableDistanceCorrection(bool enable) {
    m_bEnableDistanceCorrection = enable;
}

int16_t GeneralParser::GetVecticalAngle(int channel) {
  return m_vEleCorrection[channel];
}

bool GeneralParser::IsNeedFrameSplit(uint16_t azimuth) {
  if (abs(azimuth - m_u16LastAzimuth) > kAzimuthTolerance &&
        m_u16LastAzimuth != 0 ) {
      return true;
    }
  return false;
}

int64_t GeneralParser::GetMicroLidarTimeU64(const uint8_t* utc, int size, uint32_t timestamp) const {
  if (size != 6) {
    printf("GetMicroLidarTimeU64: array utc size is not 6 Error\n");
    return -1;
  }

  if (utc[0] != 0) {
    struct tm t = {0};
    t.tm_year = utc[0];
    if (t.tm_year >= 200) {
      t.tm_year -= 100;
    }
    t.tm_mon = utc[1] - 1;
    t.tm_mday = utc[2];
    t.tm_hour = utc[3];
    t.tm_min = utc[4];
    t.tm_sec = utc[5];
    t.tm_isdst = 0;
    return (mktime(&t)) * 1000000 + timestamp;
  }
  else {
    uint32_t utc_time_big = *(uint32_t*)(&utc[0] + 2);
    int unix_second = ((utc_time_big >> 24) & 0xff) |
            ((utc_time_big >> 8) & 0xff00) |
            ((utc_time_big << 8) & 0xff0000) |
            ((utc_time_big << 24));
    return unix_second * 1000000 + timestamp;
  }
}

void GeneralParser::PrintDwPoint(const dwLidarPointXYZI* point) {
  printf("x:%f, y:%f, z:%f, intensity:%f \n", point->x, point->y, point->z, point->intensity);
}

void GeneralParser::PrintDwPoint(const dwLidarPointRTHI* point) {
  printf("theta:%f, phi:%f, radius:%f, intensity:%f \n", point->theta, point->phi, point->radius, point->intensity);
}

void GeneralParser::PrintDwPacket(const dwLidarDecodedPacket *packet) {
  printf("hostTimestamp=%ld, sensorTimestamp=%ld, duration=%ld, maxPoints=%d, nPoints=%d \n",
          packet->hostTimestamp, packet->sensorTimestamp, packet->duration, packet->maxPoints, packet->nPoints);
  printf("minHorizontalAngleRad=%f, maxHorizontalAngleRad=%f, minVerticalAngleRad=%f, maxVerticalAngleRad=%f \n",
          packet->minHorizontalAngleRad, packet->maxHorizontalAngleRad, packet->minVerticalAngleRad, packet->maxVerticalAngleRad); 
}