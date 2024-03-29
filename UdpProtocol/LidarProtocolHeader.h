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

#include "PlatUtils.h"
#ifndef LIDAR_PROTOCOL_HEADER_H
#define LIDAR_PROTOCOL_HEADER_H

#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

static bool IsLittleEndian() {
  const int a = 1;
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&a);

  return *p == 1 ? true : false;
}

template <typename T>
T little_to_native(T data) {
  T out = 0;
  if (IsLittleEndian()) {
    out = data;
  } else {
    unsigned char *pSrc = reinterpret_cast<unsigned char *>(&data +
                                                            sizeof(data) - 1),
                  *pDst = reinterpret_cast<unsigned char *>(&out);
    for (uint32_t i = 0; i < sizeof(data); i++) {
      *pDst++ = *pSrc--;
    }
  }
  return out;
}

struct HS_LIDAR_PRE_HEADER {
  static const uint16_t kDelimiter = 0xffee;
  // major version
  static const uint8_t kME = 0x01;  // mechanical lidar
  static const uint8_t kGT = 0x02;
  static const uint8_t kQT = 0x03;
  static const uint8_t kST = 0x04;
  static const uint8_t kFT = 0x05;
  static const uint8_t kXT = 0x06;

  // minor version
  static const uint8_t kV1 = 0x01;  // reserved
  static const uint8_t kV2 = 0x02;  // used by P128 series / POROS
  static const uint8_t kV3 = 0x03;  // used by P128 series
  static const uint8_t kV4 = 0x04;  // used by P128 series

  // status info version
  static const uint8_t kStatusInfoV0 = 0;

  uint16_t m_u16Delimiter;
  uint8_t m_u8VersionMajor;
  uint8_t m_u8VersionMinor;
  uint8_t m_u8StatusInfoVersion;
  uint8_t m_u8Reserved1;

  bool IsValidDelimiter() const {
    return little_to_native(m_u16Delimiter) == kDelimiter;
  }
  uint8_t GetDelimiter() const { return little_to_native(m_u16Delimiter); }
  uint8_t GetVersionMajor() const { return m_u8VersionMajor; }
  uint8_t GetVersionMinor() const { return m_u8VersionMinor; }
  uint8_t GetStatusInfoVersion() const { return m_u8StatusInfoVersion; }

  void Init(uint8_t u8VerMajor, uint8_t u8VerMinor, 
            uint8_t u8StatusInfoVer = kStatusInfoV0) {
    m_u16Delimiter = 0xffee;
    m_u8VersionMajor = u8VerMajor;
    m_u8VersionMinor = u8VerMinor;
    m_u8StatusInfoVersion = u8StatusInfoVer;
  }

  void Print() const {
    printf("HS_LIDAR_PRE_HEADER:\n");
    printf("Delimiter:%02x, valid:%d, Ver Major: %02x, minor: %02x, "
           "StatusInfoVer:%02x\n",
           GetDelimiter(), IsValidDelimiter(), GetVersionMajor(),
           GetVersionMinor(), GetStatusInfoVersion());
  }
} PACKED;

#endif
