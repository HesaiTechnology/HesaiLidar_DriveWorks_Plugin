/**
 * @file HsLidarQTV2.h
 * @brief 禾赛QT128 的UDP协议
 * @version 0.1
 * @date 2022-10-13
 * 
 * @copyright Copyright (c) 2022 Hesai Technology Co., Ltd
 * 
 */
#ifndef HS_LIDAR_QT_V2_H
#define HS_LIDAR_QT_V2_H

#include <LidarProtocolHeader.h>

#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

struct ReservedInfo1 {
  uint16_t m_u16Sts;
  uint8_t m_u8ID;

  uint8_t GetID() const { return m_u8ID; }
  uint16_t GetData() const { return little_to_native(m_u16Sts); }
} PACKED;

struct ReservedInfo2 {
  uint16_t m_u16Unused;
} PACKED;

struct ReservedInfo3 {
  uint16_t m_u16Sts;
  uint8_t m_u8ID;
  uint8_t GetID() const { return m_u8ID; }
  uint16_t GetData() const { return little_to_native(m_u16Sts); }
} PACKED;

struct HS_LIDAR_BODY_AZIMUTH_QT_V2 {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_QT_V2: azimuth:%u\n", GetAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_QT_V2 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;
  uint8_t m_u8Confidence;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  uint8_t GetConfidenceLevel() const { return m_u8Confidence; }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_QT_V2: ");
    printf("Dist:%u, Reflectivity: %u, confidenceLevel:%u\n", GetDistance(),
           GetReflectivity(), GetConfidenceLevel());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_QT_V2: ");
    printf("Dist:%u, Reflectivity: %u, n", GetDistance(), GetReflectivity());
  }
} PACKED;

struct HS_LIDAR_BODY_CRC_QT_V2 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_BODY_CRC_QT_V2:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_FUNCTION_SAFETY {
  uint8_t m_u8FSVersion;
  uint8_t m_u8State;
  uint8_t m_u8Code;
  uint16_t m_u16OutCode;
  uint8_t m_u8Reserved[8];
  HS_LIDAR_BODY_CRC_QT_V2 m_crc;
} PACKED;

struct HS_LIDAR_TAIL_QT_V2 {
  // shutdown flag, bit 0
  static const uint8_t kShutdown = 0x01;

  // return mode
  static const uint8_t kFirstReturn = 0x33;
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kLastAndStrongestReturn = 0x39;
  static const uint8_t kFirstAndLastReturn = 0x3b;
  static const uint8_t kFirstAndStrongestReturn = 0x3c;
  static const uint8_t kStrongestAndSecondReturn = 0x3e;
  static const uint8_t kSecondReturn = 0x34;
  static const uint8_t kFirstAndSecondReturn = 0x3a;

  ReservedInfo1 m_reservedInfo1;
  ReservedInfo2 m_reservedInfo2;
  uint8_t m_u8ModeFlag;
  ReservedInfo3 m_reservedInfo3;
  uint16_t m_u16AzimuthFlag;
  uint8_t m_u8WorkingMode;
  uint8_t m_u8ReturnMode;
  uint16_t m_u16MotorSpeed;
  // 年月日 时分秒
  uint8_t m_u8UTC[6];
  uint32_t m_u32Timestamp;
  uint8_t m_u8FactoryInfo;

  uint8_t GetStsID1() const { return m_reservedInfo1.GetID(); }
  uint16_t GetData1() const { return m_reservedInfo1.GetData(); }
  uint8_t GetStsID3() const { return m_reservedInfo3.GetID(); }
  uint16_t GetData3() const { return m_reservedInfo3.GetData(); }
  uint8_t GetModeFlag() const { return m_u8ModeFlag; }
  uint16_t GetMotorSpeed() const { return little_to_native(m_u16MotorSpeed); }
  uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }
  uint8_t GetReturnMode() const { return m_u8ReturnMode; }
  bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
  bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
  bool IsLastAndStrongestReturn() const {
    return m_u8ReturnMode == kLastAndStrongestReturn;
  }
  uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }
  uint8_t GetUTCData(uint8_t index) const {
    return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_QT_V2: ");
    printf(
        "sts1:%d, data1:%d, motorSpeed:%u, timestamp:%u, returnMode:0x%02x, "
        "factoryInfo:0x%02x, utc:%u %u %u %u %u %u,\n",
        GetStsID1(), GetData1(), GetMotorSpeed(), GetTimestamp(),
        GetReturnMode(), GetFactoryInfo(), GetUTCData(0), GetUTCData(1),
        GetUTCData(2), GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }

} PACKED;

struct HS_LIDAR_TAIL_SEQ_NUM_QT_V2 {
  uint32_t m_u32SeqNum;

  uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }
  static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }

  void CalPktLoss() const {
    static uint32_t u32StartSeqNum = m_u32SeqNum;
    static uint32_t u32LastSeqNum = m_u32SeqNum;
    static int32_t u32LossCount = 0;
    static uint32_t u32StartTime = GetMicroTickCount();
    if (m_u32SeqNum - u32LastSeqNum > 1) {
      u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
      // if (m_u32SeqNum - u32LastSeqNum - 1 > 1000)
      // printf("%d,  %u, %u\n", m_u32SeqNum - u32LastSeqNum - 1, u32LastSeqNum,
      // m_u32SeqNum);
    }

    // print log every 10s
    if (GetMicroTickCount() - u32StartTime >= 10 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", u32LossCount,
             m_u32SeqNum - u32StartSeqNum);
      u32LossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = m_u32SeqNum;
    }

    u32LastSeqNum = m_u32SeqNum;
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_SEQ_NUM_QT_V2:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} PACKED;

struct HS_LIDAR_TAIL_CRC_QT_V2 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_TAIL_CRC_QT_V2:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_CYBER_SECURITY_QT_V2 {
  uint8_t m_u8Signature[32];

  uint8_t GetSignatureData(uint8_t index) const {
    return m_u8Signature[index < sizeof(m_u8Signature) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_CYBER_SECURITY_QT_V2:\n");
    for (uint8_t i = 0; i < sizeof(m_u8Signature); i++)
      printf("Signature%d:%d, ", i, GetSignatureData(i));
    printf("\n");
  }
} PACKED;

struct HS_LIDAR_CYBER_SECURITYQ_QT_V2 {
  uint8_t m_u8Signature[32];
} PACKED;

struct HS_LIDAR_HEADER_QT_V2 {
  static const uint8_t kSequenceNum = 0x01;
  static const uint8_t kIMU = 0x02;
  static const uint8_t kFunctionSafety = 0x04;
  static const uint8_t kCyberSecurity = 0x08;
  static const uint8_t kConfidenceLevel = 0x10;
  static const uint8_t kSlope = 0x20;
  static const uint8_t kSelfDefine = 0x40;

  static const uint8_t kDistUnit = 0x04;
  static const uint8_t kFirstBlockLastReturn = 0x01;
  static const uint8_t kFirstBlockStrongestReturn = 0x02;

  uint8_t m_u8LaserNum;
  uint8_t m_u8BlockNum;
  uint8_t m_u8EchoCount;
  uint8_t m_u8DistUnit;
  uint8_t m_u8EchoNum;
  uint8_t m_u8Status;

  uint8_t GetLaserNum() const { return m_u8LaserNum; }
  uint8_t GetBlockNum() const { return m_u8BlockNum; }
  double GetDistUnit() const { return m_u8DistUnit / 1000.f; }
  uint8_t GetEchoCount() const { return m_u8EchoCount; }
  bool IsFirstBlockLastReturn() const {
    return m_u8EchoCount == kFirstBlockLastReturn;
  }
  bool IsFirstBlockStrongestReturn() const {
    return m_u8EchoCount == kFirstBlockStrongestReturn;
  }
  uint8_t GetEchoNum() const { return m_u8EchoNum; }
  bool HasSeqNum() const { return m_u8Status & kSequenceNum; }
  bool HasFunctionSafety() const { return m_u8Status & kFunctionSafety; }
  bool HasCyberSecurity() const { return m_u8Status & kCyberSecurity; }
  bool HasConfidenceLevel() const { return m_u8Status & kConfidenceLevel; }
  bool HasSlope() const { return m_u8Status & kSlope; }
  bool HasSelfDefine() const { return m_u8Status & kSelfDefine; }

  uint16_t GetPacketSize() const {
    return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_QT_V2) +
           (sizeof(HS_LIDAR_BODY_AZIMUTH_QT_V2) +
            (HasConfidenceLevel()
                 ? sizeof(HS_LIDAR_BODY_CHN_UNIT_QT_V2)
                 : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_QT_V2)) *
                GetLaserNum()) *
               GetBlockNum() +
           sizeof(HS_LIDAR_BODY_CRC_QT_V2) +
           (HasFunctionSafety() ? sizeof(HS_LIDAR_FUNCTION_SAFETY) : 0) +
           sizeof(HS_LIDAR_TAIL_QT_V2) +
           (HasSeqNum() ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_QT_V2) : 0) +
           sizeof(HS_LIDAR_TAIL_CRC_QT_V2) +
           (HasCyberSecurity() ? sizeof(HS_LIDAR_CYBER_SECURITY_QT_V2) : 0);
  }

  void Print() const {
    printf("HS_LIDAR_HEADER_QT_V2: ");
    printf(
        "laserNum:%02u, blockNum:%02u, DistUnit:%g, EchoCnt:%02u, "
        "EchoNum:%02u, HasSeqNum:%d\n",
        GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(),
        GetEchoNum(), HasSeqNum());
  }
} PACKED;

#endif
