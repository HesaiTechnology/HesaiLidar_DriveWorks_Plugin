#ifndef UDP4_3_PARSER_H_
#define UDP4_3_PARSER_H_

#define MAX_AZI_LEN (36000 * 256)
#define CIRCLE_ANGLE (36000)
#define CORRECTION_AZIMUTH_STEP (200)
#define CORRECTION_AZIMUTH_NUM (180)
#define FINE_AZIMUTH_UNIT (256)
#define AZIMUTH_UNIT (25600.0f)
#define AT128_LASER_NUM (128)
#define PANDAR_AT128_EDGE_AZIMUTH_OFFSET (7500)
#define PANDAR_AT128_EDGE_AZIMUTH_SIZE (1600)

#include "GeneralParser.h"
#include "PointCloudType.h"

struct PandarATCorrectionsHeader {
  uint8_t delimiter[2];
  uint8_t version[2];
  uint8_t channel_number;
  uint8_t mirror_number;
  uint8_t frame_number;
  uint8_t frame_config[8];
  uint8_t resolution;
};
static_assert(sizeof(PandarATCorrectionsHeader) == 16, "");

struct PandarATFrameInfo {
  uint32_t start_frame[8];
  uint32_t end_frame[8];
  int32_t azimuth[AT128_LASER_NUM];
  int32_t elevation[AT128_LASER_NUM];
  std::array<float, MAX_AZI_LEN> sin_map;
  std::array<float, MAX_AZI_LEN> cos_map;
};

struct PandarATCorrections {
 public:
  PandarATCorrectionsHeader header;
  uint16_t start_frame[8];
  uint16_t end_frame[8];
  int16_t azimuth[AT128_LASER_NUM];
  int16_t elevation[AT128_LASER_NUM];
  int8_t azimuth_offset[CIRCLE_ANGLE];
  int8_t elevation_offset[CIRCLE_ANGLE];
  uint8_t SHA256[32];
  PandarATFrameInfo l;  // V1.5
  std::array<float, MAX_AZI_LEN> sin_map;
  std::array<float, MAX_AZI_LEN> cos_map;
  PandarATCorrections() {
    for (int i = 0; i < MAX_AZI_LEN; ++i) {
      sin_map[i] = std::sin(2 * i * M_PI / MAX_AZI_LEN);
      cos_map[i] = std::cos(2 * i * M_PI / MAX_AZI_LEN);
    }
  }
  static const int STEP = CORRECTION_AZIMUTH_STEP;
  int8_t getAzimuthAdjust(uint8_t ch, uint16_t azi) const {
    unsigned int i = std::floor(1.f * azi / STEP);
    unsigned int l = azi - i * STEP;
    float k = 1.f * l / STEP;
    return round((1 - k) * azimuth_offset[ch * CORRECTION_AZIMUTH_NUM + i] +
                 k * azimuth_offset[ch * CORRECTION_AZIMUTH_NUM + i + 1]);
  }
  int8_t getElevationAdjust(uint8_t ch, uint16_t azi) const {
    unsigned int i = std::floor(1.f * azi / STEP);
    unsigned int l = azi - i * STEP;
    float k = 1.f * l / STEP;
    return round((1 - k) * elevation_offset[ch * CORRECTION_AZIMUTH_NUM + i] +
                 k * elevation_offset[ch * CORRECTION_AZIMUTH_NUM + i + 1]);
  }
  static const int STEP3 = CORRECTION_AZIMUTH_STEP * FINE_AZIMUTH_UNIT;
  int8_t getAzimuthAdjustV3(uint8_t ch, uint32_t azi) const {
    unsigned int i = std::floor(1.f * azi / STEP3);
    unsigned int l = azi - i * STEP3;
    float k = 1.f * l / STEP3;
    return round((1 - k) * azimuth_offset[ch * CORRECTION_AZIMUTH_NUM + i] +
                 k * azimuth_offset[ch * CORRECTION_AZIMUTH_NUM + i + 1]);
  }
  int8_t getElevationAdjustV3(uint8_t ch, uint32_t azi) const {
    unsigned int i = std::floor(1.f * azi / STEP3);
    unsigned int l = azi - i * STEP3;
    float k = 1.f * l / STEP3;
    return round((1 - k) * elevation_offset[ch * CORRECTION_AZIMUTH_NUM + i] +
                 k * elevation_offset[ch * CORRECTION_AZIMUTH_NUM + i + 1]);
  }
};

class Udp4_3_Parser : public GeneralParser {
 public:
  Udp4_3_Parser();
  virtual ~Udp4_3_Parser();

  virtual dwStatus getDecoderConstants(_dwSensorLidarDecoder_constants* constants) override;
  
  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* data, dwLidarPointRTHI* pointRTHI) override;     
  
  // 从PandarATCorrections中获取
  int16_t GetVecticalAngle(int channel) override;
  // 保存获取的角度文件-子类私有
  PandarATCorrections m_PandarAT_corrections;

private:
  /**
   * @brief 将角度文件的字节流解析，保存在类的m_PandarAT_corrections
   * 
   * @param correction_string 字节流
   * @return int 错误码
   */
  int ParseCorrectionString(char *correction_string);

};

#endif  // UDP4_3_PARSER_H_
