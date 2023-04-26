/**
 * @file Udp3_2_Parser.h
 * @brief 禾赛QT128雷达的UDP包解析
 * @version 0.1
 * @date 2022-10-13
 * 
 * @copyright Copyright (c) 2022 Hesai Technology Co., Ltd
 * 
 */
#ifndef UDP3_2_PARSER_H_
#define UDP3_2_PARSER_H_

// 角度修正文件的精度是0.001, 360 * 1000获得分度, 此时只能用int_32，单位/1000
#define HS_LIDAR_QT128_AZIMUTH_SIZE (360000)
#define HS_LIDAR_QT128_AZIMUTH_UNIT (1000)
#define HS_LIDAR_QT128_LASER_NUM (128)
#define HS_LIDAR_QT128_LOOP_NUM (4)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG (0.0354)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT (-0.0072)

#include "GeneralParser.h"
#include "HsLidarQTV2.h"
#include "PointCloudType.h"

// QT128的校正文件，通道校正文件, 也没啥用，全123456就和默认值一样
struct PandarQTChannelConfig {
 public:
  uint16_t m_u16Sob;
  uint8_t m_u8MajorVersion;
  uint8_t m_u8MinVersion;
  uint8_t m_u8LaserNum;
  uint8_t m_u8BlockNum;
  std::vector<std::vector<int>> m_vChannelConfigTable;
  std::string m_sHashValue;
  bool m_bIsChannelConfigObtained;
};

class Udp3_2_Parser : public GeneralParser {
 public:
  Udp3_2_Parser();
  virtual ~Udp3_2_Parser();

  dwStatus getDecoderConstants(_dwSensorLidarDecoder_constants* constants) override;
  
  /**
   * @brief 从校正文件中获得两组水平校正和垂直校正量
   *        对如下两数组赋值并初始化 m_vEleCorrection m_vAziCorrection
   *        QT128的雷达还有需要解析 firetime channelconfig
   * 
   * @param correction_content 读到的字节流
   * @return int 0为正常，其他数字错误
   */
  // virtual int ParseCorrectionString(char *correction_content) override;
  
  /**
   * @brief 解析获得单个包
   * 
   * @param pointRTHI 不是指一个point，而是一个二维的数组
   */
  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) override;     

  /**
   * @brief 获取每线对应的垂直方位角
   * 
   * @param channel 0-127 QT128有128线
   * @return int16_t 1.223度返回值为 1223，单位/1000
   */
  int16_t GetVecticalAngle(int channel) override;

  /**
   * @brief 为了初始化m_vQT128Firetime
   */
  virtual int LoadFiretimesString(const char *firetimes) override;
  virtual void LoadFiretimesFile(std::string firetimes_path) override;
  
  /**
   * @brief 初始化m_PandarQTChannelConfig QT雷达特有
   * 
   * @param[in] channelconfig 实时雷达获取的字节流，或从文本中读取的 
   * @return int 0成功 -1失败
   */
  virtual int LoadChannelConfigString(const char *channelconfig) override;
  virtual void LoadChannelConfigFile(std::string channel_config_path) override;

 private:
  /**
   * @brief 计算效率上 必须返回int类型
   * 
   * @param laserId 雷达线数 0-127
   * @param speed 雷达转速
   * @param loopIndex ？？从包尾获取，与回波模式两参数相关
   * @return double 
   */
  virtual double GetFiretimesCorrection(int laserId, double speed, int loopIndex);

  // 需通过函数初始化：LoadFiretimesString
  std::array<std::array<float, HS_LIDAR_QT128_LASER_NUM>, HS_LIDAR_QT128_LOOP_NUM> m_vQT128Firetime;
  // 需通过函数初始化：LoadChannelConfigString
  PandarQTChannelConfig m_PandarQTChannelConfig;
  
  std::array<float, HS_LIDAR_QT128_AZIMUTH_SIZE> sin_map;
  std::array<float, HS_LIDAR_QT128_AZIMUTH_SIZE> cos_map;

  // Only for QT128 and etc，not for AT128
  std::vector<double> m_vFiretimeCorrection;
};

#endif  // UDP3_2_PARSER_H_
