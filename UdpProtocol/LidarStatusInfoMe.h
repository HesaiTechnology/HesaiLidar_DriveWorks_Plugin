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

 
/*
 * @b Description: This file defines Mechanical Lidar diagnosis information, e.g QT128 P128.
*/

#ifndef HS_LIDAR_STATU_INFO_ME_H
#define HS_LIDAR_STATU_INFO_ME_H
#include <LidarProtocolHeader.h>

#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

// reserved1
struct strUpperBoardInfo {
  uint16_t m_u16Sts;
  uint8_t m_u8ID;
  uint8_t GetID() const { return m_u8ID; }
  uint16_t GetData() const { return little_to_native(m_u16Sts); }

  void Print() const {
    printf("upperBoard ID:%u, STS:0x%02x\n", m_u8ID, m_u16Sts);
  }

} PACKED;

// reserved2
struct strLowerBoardInfo {
  uint16_t m_u16Sts;
  uint8_t m_u8ID;
  uint8_t GetID() const { return m_u8ID; }
  uint16_t GetData() const { return little_to_native(m_u16Sts); }

  void Print() const {
    printf("lowerBoard ID:%u, STS:0x%02x\n", m_u8ID, m_u16Sts);
  }

} PACKED;

// reserved3
struct strMonitorInfo {
  uint16_t m_u16Sts;
  uint8_t m_u8ID;
  uint8_t GetID() const { return m_u8ID; }
  uint16_t GetData() const { return little_to_native(m_u16Sts); }

  void Print() const {
    printf("monitorInfo ID:%u, STS:0x%02x\n", m_u8ID, m_u16Sts);
  }
} PACKED;

#endif
