#include "GeneralParser.h"

const double GeneralParser::kLightCoefficient = 0.1498527;
const double GeneralParser::kConvertWidthUnit = 0.1498527 / 256;
const std::string GeneralParser::kLidarIPAddr("192.168.1.201");
const double GeneralParser::kDualWidthUnit = 0.03125;        // 1/32ns
const double GeneralParser::kDistUnitWidthWithDiff = 0.008;  // 8mm
const double GeneralParser::kWidthUnitWithDiff = 0.0625;     // 1/16ns
int GeneralParser::maxAzimuthLen = 36000;                    // 360°*100

GeneralParser::GeneralParser() {
  m_iMotorSpeed = 0;
  m_iReturnMode = 0;

  m_u16FramesSize = kFramesNumMax;
  m_u16FrameStartAzimuth = kFrameStartAzimuth;
  m_u16FrameEndAzimuth = kFrameEndAzimuth;
  m_u16LastAzimuth = 0;
  m_vFrames.clear();
  m_vFrames.reserve(m_u16FramesSize);
  m_currentFrame.clear();
  m_currentFrame.reserve(kPackageNumMax);
  m_iMaxPackageNum = kPackageNumMax;
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

  // 该函数必须在子类中重写，基类中仅定义了模板函数
  int ret = ParseCorrectionString(buffer);
  delete[] buffer;
  if (ret != 0) {
    printf("Parse local correction file Error\n");
  } 

  return ret;
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

void GeneralParser::SetEnableFireTimeCorrection(bool enable){
    m_bEnableFireTimeCorrection = enable;
}

void GeneralParser::SetEnableDistanceCorrection(bool enable){
    m_bEnableDistanceCorrection = enable;
}

bool GeneralParser::IsNeedFrameSplit(uint16_t azimuth, int field) {
  // field = field <= 0 ? m_PandarAT_corrections.header.frame_number - 1 : field - 1;
  // int start_frame = m_PandarAT_corrections.l.start_frame[field] / 256.0f;
  (void) field;
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