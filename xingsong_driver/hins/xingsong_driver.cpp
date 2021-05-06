#include "hins/xingsong_driver.h"

namespace hins {

XingSongDriver::XingSongDriver(const ConnectionAddress &conn_info)
  : conn_info_(conn_info)
  , data_receiver_ptr_(nullptr)
{
  guard_thread_ = std::thread(&XingSongDriver::RunMain, this);
}

XingSongDriver::~XingSongDriver()
{
  if(data_receiver_ptr_){
    delete data_receiver_ptr_;
  }
}

bool XingSongDriver::StartCapturingTCP()
{
  if(data_receiver_ptr_){
    delete data_receiver_ptr_;
  }

  data_receiver_ptr_ = new LaserDataReceiver(conn_info_);
  if(!data_receiver_ptr_->IsConnected())
    return false;

  if(data_receiver_ptr_->SyncWrite() > 0){
    return true;
  }
  return false;
}

void XingSongDriver::RunMain()
{
  if(StartCapturingTCP()){
    std::cout << "兴颂雷达启动成功." << std::endl;
  }else{
    std::cout << "兴颂雷达启动失败，请检查网络连接." << std::endl;
  }

  while(true) // @todo: stop loop condition need to modified
  {
    if(IsConnected())
    {
      //
    }
    else
    {
      while(IsConnected()){
        std::cout << "激光雷达断线，正在重新连接 ... " << std::endl;
        if(StartCapturingTCP()){
          std::cout << "激光雷达重新连接成功." << std::endl;
        }
        usleep(1000 * 50); // 50ms
      }
    }

    usleep(1000*25); // 25ms
  }

  Disconnect();
}

}
