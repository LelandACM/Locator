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

/**
 * @brief 兴颂激光雷达驱动
 */

#pragma once

#include "hins/laser_data_receiver.h"

namespace hins {

class XingSongDriver;
using XingSongDriverHdr = std::shared_ptr<XingSongDriver>;

class XingSongDriver{
public:
  XingSongDriver(const ConnectionAddress& conn_info);
  ~XingSongDriver();

  bool StartCapturingTCP();

  ScanData GetFullScan(){
    if(data_receiver_ptr_)
      return data_receiver_ptr_->GetFullScan();
    else
      return ScanData();
  }

  bool Connect(){
    return data_receiver_ptr_->Connect();
  }

  bool IsConnected(){
    return data_receiver_ptr_->IsConnected();
  }

  void Disconnect(){
    data_receiver_ptr_->Disconnect();
  }

private:
  void RunMain();

private:
  ConnectionAddress conn_info_;
  LaserDataReceiverPtr data_receiver_ptr_;
  std::thread guard_thread_;
};

}
