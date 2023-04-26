#ifndef GENERAL_PARSER_H_
#define GENERAL_PARSER_H_

#define CIRCLE (360000)  // correction file has 3 digits, plus 1000
#define MAX_LASER_NUM (512)

#include <semaphore.h>
#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <fstream>
#include <dw/sensors/plugins/lidar/LidarDecoder.h>
#include <dw/sensors/plugins/lidar/LidarPlugin.h>
#include "PointCloudType.h"

class GeneralParser {
 public:
  GeneralParser();
  virtual ~GeneralParser();

  /**
   * @brief 将雷达的参数信息传入Driveworks
   * 
   * @param[out] constants 对该外部传入的指针进行赋值
   * @return dwStatus 
   */
  virtual dwStatus getDecoderConstants(_dwSensorLidarDecoder_constants* constants) = 0;

  /**
   * @brief Decode the data buffer to a single packet
   * 
   * @param[out] output driveworks规定的点云格式包
   * @param[in] buffer data buffer of UDP
   * @param[in] length 字节流长度
   * @param[out] data xyzi坐标点数据
   * @param[out] pointRTHI rthi格式的坐标点数据
   * @return dwStatus 
   */
  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) = 0;     
  
  virtual int32_t CalibrateAzimuth(int32_t azimuth, unsigned int laserId);

  dwStatus ComputeDwPoint(dwLidarPointXYZI& pointXYZI, dwLidarPointRTHI& pointRTHI, double radius, int32_t elevation, int32_t azimuth, uint8_t intensity);

  /**
   * @brief 读取本地的校正文件，获字节流调LoadCorrectionString实际解析
   * 
   * @param correction_path 校正文件的路径
   */
  virtual int LoadCorrectionFile(std::string correction_path);

  /**
   * @brief Parse the correction byte into typical data structure used in the derived class
   * Global flag m_bGetCorrectionFile is set to true when it decodes succussfully
   * Used in function 'LoadCorrectionFile', need to be override in the derived class
   * QT128 P128 are the same
   */
  virtual int ParseCorrectionString(char *correction_string);

  /**
   * @brief Decode the correction bytes that controls the sequence of laser emitting, Only for QT128 
   */
  virtual int LoadFiretimesString(const char *firetimes);

  /**
   * @brief Load and decode the firetime file to correct lidar. For QT128 it displays normally even without.
   * In the Web 192.168.0.1 you can check if the firetime of QT128 exist or not
   */
  virtual void LoadFiretimesFile(std::string firetimes_path);
  
  virtual int LoadChannelConfigString(const char *channelconfig);

  virtual void LoadChannelConfigFile(std::string channel_config_path);

  void DisableUpdateMonitorInfo();
  void SetEnableFireTimeCorrection(bool enable);
  void SetEnableDistanceCorrection(bool enable);

  // 其初始化依赖LoadCorrectionString，此前double， 1.234度，现在 1 234，单位0.001 存的是真正的垂直方位角和水平方位角，返回值得int？？
  // 32767 int_16， 必须用32了 360 000
  std::vector<int32_t> m_vEleCorrection;
  std::vector<int32_t> m_vAziCorrection;

  ////////////////// Parameters Required by Nvidia Driveworks //////////////////
  /**
   * @brief 从解析的校正文件中获取每一线的垂直角信息, 不同雷达的校正结构体文件不同，需子类中获取
   * 
   * @param channel 如128线有128个通道，
   * @return int16_t 每一线的垂直角信息
   */
  virtual int16_t GetVecticalAngle(int channel);

  /**
   * @brief 对比最近的水平方位角，决定是否要分帧,是否扫描完毕一圈，是否发生从355-0的跳跃判断
   * 
   * @param azimuth 水平方位角, pAzimuth->GetAzimuth()获得，355 * 100五位数
   * @param field 无用参数，不传or0
   * @return true 完成一个扫描周期，例如一圈360度，获得一帧数据
   * @return false 分度角仍在一个扫描周期内
   */
  bool IsNeedFrameSplit(uint16_t azimuth);

  // 是否获取到校正文件，解析成功后置true
  bool m_bGetCorrectionFile;
  // 是否双回波，数据量会大一倍
  bool m_bIsDualReturn;
  // 雷达内部转轴转速？
  uint16_t m_u16SpinSpeed;

  // Use to debug
  void PrintDwPoint(const dwLidarPointXYZI* point);
  void PrintDwPoint(const dwLidarPointRTHI* point);
  
 protected:

  /**
   * @brief 组合utc时间的数组和雷达的timestamp生成us级的16位时间戳
   * 
   * @param utc 数组内应含有6个参数，如uint8_t m_u8UTC[6];
   * @return int64_t -1获取失败, utc数组的传入大小不是6
   */
  int64_t GetMicroLidarTimeU64(const uint8_t* utc, int size, uint32_t timestamp) const;
  
  /**
   * @brief 打印除包中点云之外的其他参数，如时间戳点数信息等
   * 
   * @param packet 一个UDP包中全部dw需要的信息
   */
  void PrintDwPacket(const dwLidarDecodedPacket *packet);

  /**
   * @brief return the string vector split by char
   * To avoid introduce the boost library as cross complie of drivework failed
   *
   * @param[out] result  return the string vector
   * @param[in] line string to be split
   * @param[in] c symbol to be split, like ','
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
  
  // 用于距离校正，该函数仅QT用了，且未启用 For Pandar128
  float m_fCosAllAngle[CIRCLE];
  float m_fSinAllAngle[CIRCLE];
  int m_iAzimuthUnit = 1000;

  int m_iReturnMode;
  int m_iMotorSpeed;
  int m_iMaxPackageNum;

  static const std::string kLidarIPAddr;
  static const uint16_t kTcpPort = 9347;
  static const uint16_t kUdpPort = 2368;
  static const uint16_t kBufSize = 1500;
  static const uint16_t kChannelSize = 256;

  // 水平方位角，*100之后的
  static const uint16_t kFrameStartAzimuth = 0;
  static const uint16_t kFrameEndAzimuth = 36000;
  // 10度的意思吗？360-0时发生跳跃？通过捕捉这个跳跃判断一个scan是否结束
  // 显然QT128 和 P128定义的分度角不一样 TODO
  static const uint16_t kAzimuthTolerance = 1000;
  static const uint16_t kFramesNumMax = 10;
  static const uint16_t kPackageNumMax = 4000;

  // default udp packet buffer size, 1s recv 18000 packets at most
  static const uint32_t kMaxListSize = 18000;
  static const int32_t kThreadCount = 3;       // number of processing thread
  static const uint32_t kTimeout = 100000000;  // handle thread wait time
  static const long kNSecToSec = 1000000000;   // 1sec = 1000000000nsec

  UdpPacket m_packet;
  bool m_bBlockProcess;
  bool m_bSaveFrame;
  uint16_t m_u16FramesSize;
  uint16_t m_u16FrameStartAzimuth;
  uint16_t m_u16FrameEndAzimuth;
  // 记录上一个水平方位角用于分帧
  uint16_t m_u16LastAzimuth;

  // boost::mutex m_mutex;

  // Only for QT128 and etc，not for AT128
  std::vector<double> m_vFiretimeCorrection;
  bool m_bEnableFireTimeCorrection;
  bool m_bEnableDistanceCorrection;
};

#endif  // GENERAL_PARSER_H_
