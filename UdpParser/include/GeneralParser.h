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

#ifndef GENERAL_PARSER_H_
#define GENERAL_PARSER_H_

// correction file has 3 digits, plus 1000
#define CIRCLE (360000)
#define MAX_LASER_NUM (512)

#include <vector>
#include <string>
#include <cmath>
#include <fstream>

#include <dw/sensors/plugins/lidar/LidarDecoder.h>
#include <dw/sensors/plugins/lidar/LidarPlugin.h>

class GeneralParser {
 public:
  GeneralParser();
  virtual ~GeneralParser();

  /**
   * @brief Introduce characteristic parameters of each lidar to Driveworks, e.g. speed
   * @param[out] constants return struct stored the params
   */
  virtual dwStatus GetDecoderConstants(_dwSensorLidarDecoder_constants* constants) = 0;

  /**
   * @brief Decode the data buffer to a single packet
   * 
   * @param[out] output packet format ruled by driveworks
   * @param[in] buffer data buffer of UDP
   * @param[in] length length of data byte
   * @param[out] pointXYZI return xyzi coordinate
   * @param[out] pointRTHI return rthi coordinate
   */
  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) = 0;     
  
  /**
   * @brief Use correction file to calibrate the azimuth of each laser channel
   * @return int32_t unit is 1000 360 000
  */
  virtual int32_t CalibrateAzimuth(int32_t azimuth, unsigned int laserID);

  /**
   * @brief Caculate the coordinates
   * @param[out] pointXYZI Decoded coordinates of points
   * @param[out] pointRTHI Decoded coordinates of points
  */
  virtual dwStatus ComputeDwPoint(dwLidarPointXYZI& pointXYZI, dwLidarPointRTHI& pointRTHI, double radius, int32_t elevation, int32_t azimuth, uint8_t intensity);

  /**
   * @brief Decode the correction bytes that controls the sequence of laser emitting, Only for QT128 
   */
  virtual int LoadFiretimesString(const char *firetimes);

  /**
   * @brief Load and decode the firetime file to correct lidar. For QT128 it displays normally even without.
   * In the Web 192.168.1.201 you can check if the firetime of QT128 exist or not
   */
  virtual void LoadFiretimesFile(std::string firetimes_path);
  
  virtual int LoadChannelConfigString(const char *channelconfig);
  virtual void LoadChannelConfigFile(std::string channel_config_path);

  /**
   * @brief Load the correction file from a local path, then call 'LoadCorrectionString' that might be overrided
   */
  virtual int LoadCorrectionFile(std::string correction_path);

  /**
   * @brief Parse the correction byte into typical data structure used in the derived class
   * Global flag m_bGetCorrectionFile is set to true when it decodes succussfully
   * Used in function 'LoadCorrectionFile', need to be override in the derived class
   * QT128 P128 are the same
   */
  virtual int ParseCorrectionString(char *correction_string);

  // Correction angle from the file has three digits, e.g. 1.234 degree is converted to 1 234
  // Must be initilized by func 'LoadCorrectionString'
  // Notice that the maximal value of int_16 is '32767', so int32_t is used to store the value
  std::vector<int32_t> m_vEleCorrection;
  std::vector<int32_t> m_vAziCorrection;

  /**
   * @brief Get the vertical angle from the decoded correction file, specify which vertical angle of the laser channel
   * @param channel 128 channel for P128 QT128
   * @return int16_t return the vertical angle of the channel
   */
  virtual int16_t GetVecticalAngle(int channel);

  /**
   * @brief Compare to the latest azimuth, decide whether a complete frame is obtained or not. e.g. 359 00 - 0 00
   * 
   * @param azimuth normally from the UDP packet, unit 100, pAzimuth->GetAzimuth(), 355 * 100
   * @return true A complete frame data is acquired 360 degree
   * @return false Current azimuth belongs to the last scan, uncomplete scan
   */
  bool IsNeedFrameSplit(uint16_t azimuth);

  // Set true if the correction file is sucessfully decoded
  bool m_bGetCorrectionFile;
  // Return two points of each laser channel if the flag is set true, default dual return for P128
  bool m_bIsDualReturn;
  // Speed of lidar rotating spin
  uint16_t m_u16SpinSpeed;

  // For debugging
  void PrintDwPoint(const dwLidarPointXYZI* point);
  void PrintDwPoint(const dwLidarPointRTHI* point);
  
 protected:

  /**
   * @brief Combine the utc and timestamps of lidar to get timestamps (us)
   * 
   * @param utc contains six parameters of utc time
   * @return int64_t return -1 if the size of utc is not six
   */
  int64_t GetMicroLidarTimeU64(const uint8_t* utc, int size, uint32_t timestamp) const;
  
  /**
   * @brief print all necessary messages except for the point clouds, e.g. timestamp
   */
  void PrintDwPacket(const dwLidarDecodedPacket *packet);

  /**
   * @brief return the string vector split by char
   * To avoid introduce the boost library as cross complie of drivework failed
   *
   * @param[out] result  return the string vector
   * @param[in] line string to be split
   * @param[in] c symbol to be split, e.g. ','
   */
  inline void HSSplit(std::vector<std::string>& result, std::string s, char c) {
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(c)) != std::string::npos) {
        token = s.substr(0, pos);
        result.push_back(token);
        s.erase(0, pos + 1);
    }
    result.push_back(s);
  }

  inline float64_t deg2Rad(float64_t deg)
  {
      return deg * 0.01745329251994329575;
  }

  inline float64_t rad2Deg(float64_t rad)
  {
      return rad * 57.29577951308232087721;
  }
  
  // store the value of sin/cos to speed up the computing
  float m_fCosAllAngle[CIRCLE];
  float m_fSinAllAngle[CIRCLE];
  // unit of aziumth/elevation from correction file
  int m_iAziCorrUnit = 1000;

  int m_iReturnMode = 0;
  int m_iMotorSpeed = 0;

  static const std::string kLidarIPAddr;
  static const uint16_t kTcpPort = 9347;
  static const uint16_t kUdpPort = 2368;

  // default udp packet buffer size, 1s recv 18000 packets at most
  static const uint32_t kMaxListSize = 18000;
  // 1sec = 1000000000nsec
  static const long kNSecToSec = 1000000000;

  // to record the last azimuth to decide split frame or not
  uint16_t m_u16LastAzimuth = 0;
  // to judge if a complete frame data is collected, 
  // curAzimuth - m_u16LastAzimuth > kAzimuthTolerance 360-0, 10 degree, unit 100
  static const uint16_t kAzimuthTolerance = 1000;
};

#endif  // GENERAL_PARSER_H_
