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

#ifndef HS_LIDAR_ST_V3_H
#define HS_LIDAR_ST_V3_H

#include <LidarProtocolHeader.h>
#include <LidarStatusInfoMe.h>
#include "PlatUtils.h"

#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

struct HS_LIDAR_BODY_AZIMUTH_ST_V3 {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_ST_V3: azimuth:%u\n", GetAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3 {
  uint8_t m_u8FineAzimuth;

  uint8_t GetFineAzimuth() const { return little_to_native(m_u8FineAzimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3: FineAzimuth:%u\n",
           GetFineAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_NNIT_ST_V3 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;
  uint8_t m_u8Confidence;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  uint8_t GetConfidenceLevel() const { return m_u8Confidence; }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_NNIT_ST_V3:\n");
    printf("Dist:%u, Reflectivity: %u, confidenceLevel:%d\n", GetDistance(),
           GetReflectivity(), GetConfidenceLevel());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_CALIBRATION_ST_V3 {
  uint16_t m_u16Width;
  uint16_t m_u16Front;

  uint16_t GetWidth() const { return little_to_native(m_u16Width); }
  uint16_t GetFront() const { return little_to_native(m_u16Front); }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_CALIBRATION_ST_V3:\n");
    printf("Width:%u, Front: %u\n", GetWidth(), GetFront());
  }
} PACKED;

struct HS_LIDAR_BODY_CRC_ST_V3 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_BODY_CRC_ST_V3:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_TAIL_ST_V3 {
  // shutdown flag, bit 0
  static const uint8_t kShutdown = 0x01;

  // return mode
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;

  strUpperBoardInfo m_upperBoardInfo;
  strLowerBoardInfo m_lowerBoardInfo;
  uint8_t m_u8Shutdown;
  strMonitorInfo m_monitorInfo;
  uint8_t m_u8UReserved[8];
  int16_t m_i16MotorSpeed;
  uint32_t m_u32Timestamp;
  uint8_t m_u8ReturnMode;
  uint8_t m_u8FactoryInfo;
  uint8_t m_u8UTC[6];
  // uint32_t m_u32SeqNum;

  uint8_t GetStsID0() const { return m_upperBoardInfo.GetID(); }
  uint16_t GetData0() const { return m_upperBoardInfo.GetData(); }

  uint8_t GetStsID1() const { return m_lowerBoardInfo.GetID(); }
  uint16_t GetData1() const { return m_lowerBoardInfo.GetData(); }

  uint8_t HasShutdown() const { return m_u8Shutdown & kShutdown; }

  uint8_t GetStsID2() const { return m_monitorInfo.GetID(); }
  uint16_t GetData2() const { return m_monitorInfo.GetData(); }

  int16_t GetMotorSpeed() const { return little_to_native(m_i16MotorSpeed); }

  uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }

  uint8_t GetReturnMode() const { return m_u8ReturnMode; }
  bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
  bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
  bool IsDualReturn() const { return m_u8ReturnMode >= kDualReturn; }

  uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }

  uint8_t GetUTCData(uint8_t index) const {
    return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
  }

  int64_t GetMicroLidarTimeU64() const {
    if (m_u8UTC[0] != 0) {
			struct tm t = {0};
			t.tm_year = m_u8UTC[0];
			if (t.tm_year >= 200) {
				t.tm_year -= 100;
			}
			t.tm_mon = m_u8UTC[1] - 1;
			t.tm_mday = m_u8UTC[2];
			t.tm_hour = m_u8UTC[3];
			t.tm_min = m_u8UTC[4];
			t.tm_sec = m_u8UTC[5];
			t.tm_isdst = 0;
			return (mktime(&t)) * 1000000 + GetTimestamp();
		}
		else {
      uint32_t utc_time_big = *(uint32_t*)(&m_u8UTC[0] + 2);
      int unix_second = ((utc_time_big >> 24) & 0xff) |
              ((utc_time_big >> 8) & 0xff00) |
              ((utc_time_big << 8) & 0xff0000) |
              ((utc_time_big << 24));
      return unix_second * 1000000 + GetTimestamp();
		}

  }

  // uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }
  // static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }

  void Print() const {
    printf("HS_LIDAR_TAIL_ST_V3:\n");
    printf(
        "sts0:%d, data0:%d, sts1:%d, data1:%d, shutDown:%d, motorSpeed:%d, "
        "timestamp:%u, returnMode:0x%02x, factoryInfo:0x%02x, utc:%u %u "
        "%u %u %u %u\n",
        GetStsID0(), GetData0(), GetStsID1(), GetData1(), HasShutdown(),
        GetMotorSpeed(), GetTimestamp(), GetReturnMode(), GetFactoryInfo(),
        GetUTCData(0), GetUTCData(1), GetUTCData(2), GetUTCData(3),
        GetUTCData(4), GetUTCData(5));
  }

} PACKED;

struct HS_LIDAR_TAIL_SEQ_NUM_ST_V3 {
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
      // printf("%d,  %u, %u\n", m_u32SeqNum - u32LastSeqNum - 1, u32LastSeqNum, m_u32SeqNum);
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
    printf("HS_LIDAR_TAIL_SEQ_NUM_ST_V3:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} PACKED;

struct HS_LIDAR_TAIL_CRC_ST_V3 {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return little_to_native(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_TAIL_CRC_ST_V3:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} PACKED;

struct HS_LIDAR_CYBER_SECURITY_ST_V3 {
  uint8_t m_u8Signature[32];

  uint8_t GetSignatureData(uint8_t index) const {
    return m_u8Signature[index < sizeof(m_u8Signature) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_CYBER_SECURITY_ST_V3:\n");
    for (uint8_t i = 0; i < sizeof(m_u8Signature); i++)
      printf("Signature%d:%d, ", i, GetSignatureData(i));
    printf("\n");
  }
} PACKED;

struct HS_LIDAR_HEADER_ST_V3 {
  static const uint8_t kSequenceNum = 0x01;
  static const uint8_t kIMU = 0x02;
  static const uint8_t kFunctionSafety = 0x04;
  static const uint8_t kCyberSecurity = 0x08;
  static const uint8_t kConfidenceLevel = 0x10;

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
  bool HasIMU() const { return m_u8Status & kIMU; }
  bool HasFuncSafety() const { return m_u8Status & kFunctionSafety; }
  bool HasCyberSecurity() const { return m_u8Status & kCyberSecurity; }
  bool HasConfidenceLevel() const { return m_u8Status & kConfidenceLevel; }

  uint16_t GetPacketSize() const {
    return sizeof(HS_LIDAR_PRE_HEADER) + sizeof(HS_LIDAR_HEADER_ST_V3) +
           (sizeof(HS_LIDAR_BODY_AZIMUTH_ST_V3) +
            sizeof(HS_LIDAR_BODY_FINE_AZIMUTH_ST_V3) +
            sizeof(HS_LIDAR_BODY_CHN_NNIT_ST_V3) * GetLaserNum()) *
               GetBlockNum() +
           sizeof(HS_LIDAR_BODY_CRC_ST_V3) + sizeof(HS_LIDAR_TAIL_ST_V3) +
           (HasSeqNum() ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_ST_V3) : 0) +
           sizeof(HS_LIDAR_TAIL_CRC_ST_V3) +
           (HasCyberSecurity() ? sizeof(HS_LIDAR_CYBER_SECURITY_ST_V3) : 0);
  }

  void Print() const {
    printf("HS_LIDAR_HEADER_ST_V3:\n");
    printf(
        "laserNum:%02u, blockNum:%02u, DistUnit:%g, EchoCnt:%02u, "
        "EchoNum:%02u, HasSeqNum:%d, HasIMU:%d, "
        "HasFuncSafety:%d, HasCyberSecurity:%d, HasConfidence:%d\n",
        GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(),
        GetEchoNum(), HasSeqNum(), HasIMU(), HasFuncSafety(),
        HasCyberSecurity(), HasConfidenceLevel());
  }
} PACKED;

#endif
