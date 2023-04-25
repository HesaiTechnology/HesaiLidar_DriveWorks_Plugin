
#ifndef HS_LIDAR_ME_V4_H
#define HS_LIDAR_ME_V4_H

#include <LidarProtocolHeader.h>
#include <LidarStatusInfoMe.h>

#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

struct HS_LIDAR_BODY_AZIMUTH_ME_V4 {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_ME_V4: azimuth:%u\n", GetAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }

  // code from fw, when udp type = 7:
  // for last return: low threshold width = distance / 2 ** 12 + reflectivity * 2 ** 4
  //                  high threshold width = distance % 2 ** 12
  // for strongest return: gate = reflectivity
  uint16_t GetLowWidth() const {
    return m_u8Reflectivity << 4 | m_u16Distance >> 12;
  }
  uint16_t GetHighWidth() const {
    return m_u16Distance & 0x0FFF;
  }
  uint8_t GetGate() const { return m_u8Reflectivity; }
  uint16_t GetFront() const { return m_u16Distance; }
  uint16_t GetLowWidthWithDiff() const {
    return m_u8Reflectivity << 3 | m_u16Distance >> 13;
  }
  uint16_t GetWidthDiff() const {
    return (m_u16Distance & 0x1FFF) >> 6;
  }
  uint16_t GetWidthTempError() const {
    return m_u16Distance & 0x03F;
  }
  uint16_t GetDistanceWithWidthDiff() const {
    return little_to_native(m_u16Distance >> 1);
  }
  uint16_t GetTempErrorSymbol() const {
    return m_u16Distance & 0x1;
  }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4:\n");
    printf("Dist:%u, Reflectivity: %u\n", GetDistance(), GetReflectivity());
  }
  void PrintMixData() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4:\n");
    printf("LowWidth:%hu, HighWidth: %hu Gate: %u Front: %hu\n",
           GetLowWidth(), GetHighWidth(), GetGate(), GetFront());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_ME_V4 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;
  uint8_t m_u8Confidence;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  uint8_t GetConfidenceLevel() const { return m_u8Confidence; }

  // code from fw, when udp type = 7:
  // for last return: low threshold width = distance / 2 ** 12 + reflectivity * 2 ** 4
  //                  high threshold width = distance % 2 ** 12
  // for strongest return: gate = reflectivity
  uint16_t GetLowWidth() const {
    return m_u8Reflectivity << 4 | m_u16Distance >> 12;
  }
  uint16_t GetHighWidth() const {
    return m_u16Distance & 0x0FFF;
  }
  uint8_t GetGate() const { return m_u8Reflectivity; }
  uint16_t GetFront() const { return m_u16Distance; }
  uint16_t GetLowWidthWithDiff() const {
    return m_u8Reflectivity << 3 | m_u16Distance >> 13;
  }
  uint16_t GetWidthDiff() const {
    return (m_u16Distance & 0x1FFF) >> 6;
  }
  uint16_t GetWidthTempError() const {
    return m_u16Distance & 0x03F;
  }
  uint16_t GetDistanceWithWidthDiff() const {
    return little_to_native(m_u16Distance >> 1);
  }
  uint16_t GetTempErrorSymbol() const {
    return m_u16Distance & 0x1;
  }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_ME_V4:\n");
    printf("Dist:%u, Reflectivity: %u, confidenceLevel:%d\n", GetDistance(),
           GetReflectivity(), GetConfidenceLevel());
  }
  void PrintMixData() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_ME_V4:\n");
    printf("LowWidth:%hu, HighWidth: %hu Gate: %u Front: %hu\n",
           GetLowWidth(), GetHighWidth(), GetGate(), GetFront());
  }
} PACKED;

struct HS_LIDAR_BODY_CRC_ME_V4 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_BODY_CRC_ME_V4:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_CYBER_SECURITY_ME_V4 {
  uint8_t m_u8Signature[32];

  uint8_t GetSignatureData(uint8_t index) const {
    return m_u8Signature[index < sizeof(m_u8Signature) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_CYBER_SECURITY_ME_V4:\n");
    for (uint8_t i = 0; i < sizeof(m_u8Signature); i++)
      printf("Signature%d:%d, ", i, GetSignatureData(i));
    printf("\n");
  }
} PACKED;

struct HS_LIDAR_FUNC_SAFETY_ME_V4 {
  static const uint8_t kLidarStateShiftBits = 5;
  static const uint8_t kLidarStateMask = 0x07;
  static const uint8_t kRollingCntShiftBits = 0;
  static const uint8_t kRollingCntMask = 0x07;
  static const uint8_t kFaultTypeCurrent = 0x08;
  static const uint8_t kFaultTypeHistory = 0x10;
  static const uint8_t kFaultNumShiftBits = 4;
  static const uint8_t kFaultNumMask = 0x0F;
  static const uint8_t kFaultIDShiftBits = 0;
  static const uint8_t kFaultIDMask = 0x0F;

  uint8_t m_u8Version;
  uint8_t m_u8Status;
  uint8_t m_u8FaultInfo;
  uint16_t m_u16FaultCode;
  uint8_t m_u8Reserved[8];
  uint32_t m_u32Crc;

  uint8_t GetVersion() const { return m_u8Version; }

  uint8_t GetLidarState() const {
    return (m_u8Status >> kLidarStateShiftBits) & kLidarStateMask;
  }
  bool IsHistoryFault() const { return m_u8Status & kFaultTypeHistory; }
  bool IsCurrentFault() const { return m_u8Status & kFaultTypeCurrent; }

  uint8_t GetRollingCnt() const {
    return (m_u8Status >> kRollingCntShiftBits) & kRollingCntMask;
  }

  uint8_t GetFaultNum() const {
    return (m_u8FaultInfo >> kFaultNumShiftBits) & kFaultNumMask;
  }
  uint8_t GetFaultID() const {
    return (m_u8FaultInfo >> kFaultIDShiftBits) & kFaultIDMask;
  }

  uint16_t GetFaultCode() const { return little_to_native(m_u16FaultCode); }

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_FUNC_SAFETY_ME_V4:\n");
    printf("ver:%u, lidarState:%u, rollingCnt:%u, isCurrFault:%u, "
           "faultNum:%u, faultID:%u, faultCode:0x%04x, crc:0x%08x\n",
           GetVersion(), GetLidarState(), GetRollingCnt(), IsCurrentFault(),
           GetFaultNum(), GetFaultID(), GetFaultCode(), GetCrc());
  }
} PACKED;

struct HS_LIDAR_TAIL_ME_V4 {
  // shutdown flag, bit 0
  static const uint8_t kHighPerformanceMode = 0x00;
  static const uint8_t kShutdown = 0x01;
  static const uint8_t kStandardMode = 0x02;
  static const uint8_t kEnergySavingMode = 0x03;

  // return mode
  static const uint8_t kFirstReturn = 0x33;
  static const uint8_t kSecondReturn = 0x34;
  static const uint8_t kThirdReturn = 0x35;
  static const uint8_t kFourthReturn = 0x36;
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;
  static const uint8_t kFirstSecondReturn = 0x3a;
  static const uint8_t kStongestFirstReturn = 0x3b;

  strUpperBoardInfo m_upperBoardInfo;
  strLowerBoardInfo m_lowerBoardInfo;
  strMonitorInfo m_monitorInfo;
  uint16_t m_u16AzimuthFlag;
  uint8_t m_u8RunningMode;
  uint8_t m_u8ReturnMode;
  uint16_t m_u16MotorSpeed;
  uint8_t m_u8UTC[6];
  uint32_t m_u32Timestamp;
  uint8_t m_u8FactoryInfo;

  uint8_t GetStsID0() const { return m_upperBoardInfo.GetID(); }
  uint16_t GetData0() const { return m_upperBoardInfo.GetData(); }
  uint8_t GetStsID1() const { return m_lowerBoardInfo.GetID(); }
  uint16_t GetData1() const { return m_lowerBoardInfo.GetData(); }
  uint8_t GetStsID2() const { return m_monitorInfo.GetID(); }
  uint16_t GetData2() const { return m_monitorInfo.GetData(); }

  uint8_t HasShutdown() const { return m_u8RunningMode == kShutdown; }
  uint8_t GetReturnMode() const { return m_u8ReturnMode; }
  uint16_t GetMotorSpeed() const { return little_to_native(m_u16MotorSpeed); }
  uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }

  bool IsFirstReturn() const { return m_u8ReturnMode == kFirstReturn; }
  bool IsSecondReturn() const { return m_u8ReturnMode == kSecondReturn; }
  bool IsThirdReturn() const { return m_u8ReturnMode == kThirdReturn; }
  bool IsFourthReturn() const { return m_u8ReturnMode == kFourthReturn; }
  bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
  bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
  bool IsDualReturn() const { return m_u8ReturnMode == kDualReturn; }
  bool IsFirstSecondReturn() const {
    return m_u8ReturnMode == kFirstSecondReturn;
  }
  bool IsStongestFirstReturn() const {
    return m_u8ReturnMode == kStongestFirstReturn;
  }

  uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }
  uint8_t GetUTCData(uint8_t index) const {
    return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_ME_V4:\n");
    printf("sts0:%d, data0:%d, sts1:%d, data1:%d, sts2:%d, data2:%d, "
           "shutDown:%d, motorSpeed:%u, timestamp:%u, returnMode:0x%02x, "
           "factoryInfo:0x%02x, utc:%u %u %u %u %u %u\n",
           GetStsID0(), GetData0(), GetStsID1(), GetData1(), GetStsID2(),
           GetData2(), HasShutdown(), GetMotorSpeed(), GetTimestamp(),
           GetReturnMode(), GetFactoryInfo(), GetUTCData(0), GetUTCData(1),
           GetUTCData(2), GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }
} PACKED;

struct HS_LIDAR_TAIL_SEQ_NUM_ME_V4 {
  uint32_t m_u32SeqNum;

  uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }
  static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }

  void CalPktLoss() const {
    static uint32_t u32StartSeqNum = m_u32SeqNum;
    static uint32_t u32LastSeqNum = m_u32SeqNum;
    static uint32_t u32LossCount = 0;
    static uint32_t u32StartTime = GetMicroTickCount();

    if (m_u32SeqNum - u32LastSeqNum - 1 > 0) {
      u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
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
    printf("HS_LIDAR_TAIL_SEQ_NUM_ME_V4:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} PACKED;

struct HS_LIDAR_TAIL_IMU_ME_V4 {
  int16_t m_i16IMUTemperature;
  uint16_t m_u16IMUAccelUnit;
  uint16_t m_u16IMUAngVelUnit;
  uint32_t m_u32IMUTimeStamp;
  int16_t m_i16IMUXAccel;
  int16_t m_i16IMUYAccel;
  int16_t m_i16IMUZAccel;
  int16_t m_i16IMUXAngVel;
  int16_t m_i16IMUYAngVel;
  int16_t m_i16IMUZAngVel;

  int16_t GetIMUTemperature() const {
    return little_to_native(m_i16IMUTemperature);
  }
  double GetIMUAccelUnit() const {
    return little_to_native(m_u16IMUAccelUnit) / 1000.f;
  }
  double GetIMUAngVelUnit() const {
    return little_to_native(m_u16IMUAngVelUnit) / 1000.f;
  }
  uint32_t GetIMUTimestamp() const {
    return little_to_native(m_u32IMUTimeStamp);
  }
  double GetIMUXAccel() const {
    return little_to_native(m_i16IMUXAccel) * GetIMUAccelUnit();
  }
  double GetIMUYAccel() const {
    return little_to_native(m_i16IMUYAccel) * GetIMUAccelUnit();
  }
  double GetIMUZAccel() const {
    return little_to_native(m_i16IMUZAccel) * GetIMUAccelUnit();
  }
  double GetIMUXAngVel() const {
    return little_to_native(m_i16IMUXAngVel) * GetIMUAngVelUnit();
  }
  double GetIMUYAngVel() const {
    return little_to_native(m_i16IMUYAngVel) * GetIMUAngVelUnit();
  }
  double GetIMUZAngVel() const {
    return little_to_native(m_i16IMUZAngVel) * GetIMUAngVelUnit();
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_IMU_ME_V4:\n");
    printf("Temp:%u, AccelUnit:%f, AngVelUnit:%f, TimeStamp:%u, XAccel:%f, "
           "YAccel:%f, ZAccel:%f, XAngVel:%f, YAngVel:%f, ZAngVel:%f\n",
           GetIMUTemperature(), GetIMUAccelUnit(), GetIMUAngVelUnit(),
           GetIMUTimestamp(), GetIMUXAccel(), GetIMUYAccel(), GetIMUZAccel(),
           GetIMUXAngVel(), GetIMUYAngVel(), GetIMUZAngVel());
  }
} PACKED;

struct HS_LIDAR_TAIL_CRC_ME_V4 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_TAIL_CRC_ME_V4:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_HEADER_ME_V4 {
  static const uint8_t kSequenceNum = 0x01;
  static const uint8_t kIMU = 0x02;
  static const uint8_t kFunctionSafety = 0x04;
  static const uint8_t kCyberSecurity = 0x08;
  static const uint8_t kConfidenceLevel = 0x10;
  static const uint8_t kDistUnit = 0x04;

  static const uint8_t kSingleReturn = 0x00;
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
  uint8_t GetEchoNum() const { return m_u8EchoNum; }
  bool HasSeqNum() const { return m_u8Status & kSequenceNum; }
  bool HasIMU() const { return m_u8Status & kIMU; }
  bool HasFuncSafety() const { return m_u8Status & kFunctionSafety; }
  bool HasCyberSecurity() const { return m_u8Status & kCyberSecurity; }
  bool HasConfidenceLevel() const { return m_u8Status & kConfidenceLevel; }
  bool IsFirstBlockLastReturn() const {
    return m_u8EchoCount == kFirstBlockLastReturn;
  }
  bool IsFirstBlockStrongestReturn() const {
    return m_u8EchoCount == kFirstBlockStrongestReturn;
  }

  uint16_t GetPacketSize() const {
    return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_ME_V4) +
           (sizeof(HS_LIDAR_BODY_AZIMUTH_ME_V4) +
            (HasConfidenceLevel()
                 ? sizeof(HS_LIDAR_BODY_CHN_UNIT_ME_V4)
                 : sizeof(HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4)) *
                GetLaserNum()) *
               GetBlockNum() +
           sizeof(HS_LIDAR_BODY_CRC_ME_V4) +
           (HasFuncSafety() ? sizeof(HS_LIDAR_FUNC_SAFETY_ME_V4) : 0) +
           (HasCyberSecurity() ? sizeof(HS_LIDAR_CYBER_SECURITY_ME_V4) : 0) +
           sizeof(HS_LIDAR_TAIL_ME_V4) + sizeof(HS_LIDAR_TAIL_CRC_ME_V4) +
           (HasSeqNum() ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_ME_V4) : 0) +
           (HasIMU() ? sizeof(HS_LIDAR_TAIL_IMU_ME_V4) : 0);
  }

  void Print() const {
    printf("HS_LIDAR_HEADER_ME_V4:\n");
    printf("laserNum:%02u, blockNum:%02u, DistUnit:%g, EchoCnt:%02u, "
           "EchoNum:%02u, HasSeqNum:%d, HasIMU:%d, "
           "HasFuncSafety:%d, HasCyberSecurity:%d, HasConfidence:%d\n",
           GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(),
           GetEchoNum(), HasSeqNum(), HasIMU(), HasFuncSafety(),
           HasCyberSecurity(), HasConfidenceLevel());
  }
} PACKED;

#endif
