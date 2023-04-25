#ifndef UDP1_4_PARSER_H_
#define UDP1_4_PARSER_H_

#define HS_LIDAR_P128_AZIMUTH_SIZE (36000)
// 角度校准文件的单位 1/100
#define HS_LIDAR_P128_AZIMUTH_UNIT (100)
#define HS_LIDAR_P128_LASER_NUM (128)

#include "GeneralParser.h"
#include "PointCloudType.h"

// For Pandar128
class Udp1_4_Parser : public GeneralParser {
 public:
  Udp1_4_Parser();
  virtual ~Udp1_4_Parser();

  dwStatus getDecoderConstants(_dwSensorLidarDecoder_constants* constants) override;
  
  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) override;

  int16_t GetVecticalAngle(int channel) override;

 private:
  const int kLaserNum = HS_LIDAR_P128_LASER_NUM;
};

#endif  // UDP1_4_PARSER_H_
