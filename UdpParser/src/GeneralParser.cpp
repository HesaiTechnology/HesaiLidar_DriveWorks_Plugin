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
#include "GeneralParser.h"

const std::string GeneralParser::kLidarIPAddr("192.168.1.201");

GeneralParser::GeneralParser() {
  for(int i = 0; i < CIRCLE; ++i) {
      m_fSinAllAngle[i] = std::sin(2 * M_PI * i / CIRCLE);
      m_fCosAllAngle[i] = std::cos(2 * M_PI * i / CIRCLE);
  }
}

GeneralParser::~GeneralParser() {
  // printf("release general Parser\n");
}

int GeneralParser::LoadCorrectionFile(std::string correction_path) {
  printf("GeneralParser: load correction file, path=%s \n", correction_path.c_str());
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
  // printf("ParseCorrectionString: parsing calibration content\n %s \n", correction_content);
  std::string correction_content_str = correction_content;
	std::istringstream ifs(correction_content_str);
	std::string line;

	// skip first line "Laser id,Elevation,Azimuth" or "eeff"
  std::getline(ifs, line);

	float elevationList[MAX_LASER_NUM], azimuthList[MAX_LASER_NUM];
	std::vector<std::string>  vfirstLine;
  HSSplit(vfirstLine, line, ',');
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
		m_vEleCorrection[i] = static_cast<int32_t> (round(elevationList[i] * m_iAziCorrUnit));
		m_vAziCorrection[i] = static_cast<int32_t> (round(azimuthList[i] * m_iAziCorrUnit));
    // printf("m_vEleCorrection, %d  m_vAziCorrection, %d \n", m_vEleCorrection[i], m_vAziCorrection[i]);
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

dwStatus GeneralParser::ComputeDwPoint(dwLidarPointXYZI& pointXYZI, dwLidarPointRTHI& pointRTHI, double radius, int32_t elevation, int32_t azimuth, uint8_t intensity) {
  // only 0 - 360 00
  double xyDistance = radius * this->m_fCosAllAngle[elevation];
  pointXYZI.x = xyDistance * this->m_fSinAllAngle[azimuth];
  pointXYZI.y = xyDistance * this->m_fCosAllAngle[azimuth];
  pointXYZI.z = radius * this->m_fSinAllAngle[elevation];
  pointXYZI.intensity = intensity;  // float type 0-1 /255.0f

  pointRTHI.radius = radius;
  // 100 is the unit!!
  pointRTHI.theta = azimuth / m_iAziCorrUnit / 180 * M_PI;
  pointRTHI.phi = elevation / m_iAziCorrUnit / 180 * M_PI;
  pointRTHI.intensity = intensity;

  return DW_SUCCESS;
}

int32_t GeneralParser::CalibrateAzimuth(int32_t azimuth, unsigned int laserID) {
  // azimuth from UDP packet has unit 100, but correction file is 1000
  int32_t result = azimuth * 10 + this->m_vAziCorrection[laserID];
  result = (CIRCLE + result) % CIRCLE;
  // printf("azimuth=%d \n", result);
  
  return result;
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