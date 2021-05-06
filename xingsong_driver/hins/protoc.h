/*********************************************************************
*
* Software License Agreement ()
*
*  Copyright (c) 2020, HINS, Inc.
*  All rights reserved.
*
* Author: Hu Liankui
* Create Date: 8/9/2020
*********************************************************************/

#pragma once

#include <array>
#include <vector>

namespace hins {

const std::array<unsigned char, 8> kStartCapture {0x52, 0x41, 0x75, 0x74, 0x6f, 0x01, 0x87, 0x80};

struct ScanData
{
  // 距离数据
  std::vector<std::uint32_t> distance_data;

  // 强度数据
  std::vector<std::uint32_t> amplitude_data;
};

#pragma pack(1)

struct XSPackageHeader{
  char head[4];
  uint16_t start_angle;      // 起始角度
  uint16_t end_angle;        // 终止角度
  uint16_t data_size;        // 这个包里面总的测量点数
  uint16_t data_position;    // 当前帧测量点位置
  uint16_t measure_size;     // 当前帧测量点数量
  uint16_t time;             // 时间标志（未启用）
};

const uint16_t kXSPackageHeadSize = sizeof(XSPackageHeader);
const uint16_t kMaxDistance = 60000;
const uint16_t kMaxIntensity = 30000;

#pragma pack()

}



