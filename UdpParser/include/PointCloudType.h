#ifndef __PANDAR_POINTCLOUD_POINT_TYPES_H
#define __PANDAR_POINTCLOUD_POINT_TYPES_H

#include <stdint.h>
#include <vector>
#include "InputSocket.h"

// struct UdpPacket {
//   uint8_t m_u8Buf[1500];
//   int16_t m_i16Len;
// };

typedef std::vector<UdpPacket> UdpFrame_t;
typedef std::vector<UdpFrame_t> UdpFrameArray_t;

struct PointCloud {
  // 水平角度
  double m_dAzimuth;
  //垂直角
  double m_dElevation; 
  // 反射率
  uint8_t m_u8Intensity;
  // 置信度
  uint8_t m_u8Confidence;
  // 距离
  double m_dDistance;
  // block id
  uint8_t m_u8BlockId;

  PointCloud(double azimuth = 0, double elevation = 0, uint8_t intensity = 0,
             uint8_t confidence = 0, double distance = 0, uint8_t blockId = 0)

      : m_dAzimuth(azimuth),
        m_dElevation(elevation),
        m_u8Intensity(intensity),
        m_u8Confidence(confidence),
        m_dDistance(distance),
        m_u8BlockId(blockId) {}
};

#endif  // __PANDAR_POINTCLOUD_POINT_TYPES_H
