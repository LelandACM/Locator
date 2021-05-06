#include "hins/laser_data_receiver.h"
#include "hins/protoc.h"
#include "utils.h"
#include <iostream>

namespace hins {

LaserDataReceiver::LaserDataReceiver(const ConnectionAddress& conn_info)
  : conn_info_(conn_info)
  , is_connected_(false)
  , inbuf_(4096)
  , instream_(&inbuf_)
  , tcp_socket_ptr_(0)
  , ring_buffer_(65536)
  , scan_data_()
  , last_data_time_(hins::Now())
{
  printf("Connecting to TCP data channel at hostname: %s, tcp_port: %d.", conn_info.GetAddress().c_str(), conn_info.GetPort());

  if(Connect()){
    std::cout << "Lidar connect success." << std::endl;;
  }else{
    std::cout << "Lidar connect failed." << std::endl;;
  }
}

LaserDataReceiver::~LaserDataReceiver()
{
  Disconnect();

  if(tcp_socket_ptr_)
    delete tcp_socket_ptr_;
}

bool LaserDataReceiver::IsConnected()
{
  return is_connected_;
}

void LaserDataReceiver::Disconnect()
{
  is_connected_ = false;
  try
  {
    if( tcp_socket_ptr_ != nullptr ){
      tcp_socket_ptr_->close();
    }

    io_service_.stop();
    if( boost::this_thread::get_id() != io_service_thread_.get_id() ){
      io_service_thread_.join();
    }
  }
  catch (std::exception& e)
  {
    printf("Exception:%s ",e.what());
  }
  printf("Disconnect.");
}

bool LaserDataReceiver::CheckConnection()
{
  if( !IsConnected() )
    return false;
  if( (hins::Now() - last_data_time_)/1000.0 > 1.0 ) // 1.0s 断线
  {
    Disconnect();
    printf("超时断线，超时时间： %f", (hins::Now() - last_data_time_)/1000.0);
    return false;
  }
  return true;
}

ScanData LaserDataReceiver::GetFullScan()
{
  std::unique_lock<std::mutex> lock(scan_mutex_);
  while( CheckConnection() && scan_data_.size() < 2 ) {
    data_notifier_.wait_for(lock, std::chrono::seconds(1));
  }

  ScanData data;
  if( scan_data_.size() >= 2 && IsConnected()){
    data = ScanData(std::move(scan_data_.front()));
    scan_data_.pop_front();
  }
  else{
    std::cout << "null data" << std::endl;
  }

  return data;
}

void LaserDataReceiver::HandleSocketRead(const boost::system::error_code &error)
{
  if(!error){
    // 1. push all data to buffer
    instream_.clear();
    while(!instream_.eof()){
      char buf[4096];
      instream_.read(buf, 4096);
      int bytes_read = instream_.gcount();
      WriteBufferBack(buf, bytes_read);
    }

    // 2. handle data in buffer
    while(HandleNextPacket()) {}

    // 3. Read data asynchronously
    boost::asio::async_read(*tcp_socket_ptr_, inbuf_, boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
  }
  else
  {
    if( error.value() != 995 )
      printf("ERROR: data connection error: %s,(%d)",error.message().c_str(),error.value());
    Disconnect();
  }
}

int LaserDataReceiver::SyncWrite()
{
  boost::system::error_code ec;
  int ret = tcp_socket_ptr_->write_some(boost::asio::buffer(kStartCapture), ec);
  if(ec)
  {
    printf("write failed：%s", boost::system::system_error(ec).what());
    ret = -1;
  }

  return ret;
}

bool LaserDataReceiver::Connect()
{
  try
  {
    // Resolve hostname/ip
    boost::asio::ip::tcp::resolver resolver(io_service_);
    boost::asio::ip::tcp::resolver::query query(conn_info_.GetAddress(), std::to_string(conn_info_.GetPort()));
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    if(nullptr == tcp_socket_ptr_){
      tcp_socket_ptr_ = new boost::asio::ip::tcp::socket(io_service_);
    }else{
      delete tcp_socket_ptr_;
      tcp_socket_ptr_ = new boost::asio::ip::tcp::socket(io_service_);
    }

    boost::system::error_code error = boost::asio::error::host_not_found;

    // Iterate over endpoints and etablish connection
    while (error && endpoint_iterator != end)
    {
      //tcp_socket_ptr_->close();
      tcp_socket_ptr_->connect(*endpoint_iterator++, error);
    }
    if (error){
      printf("Connect failed. %s", error.message().c_str());
      return false;
    }

    // Start async reading
    boost::asio::async_read(*tcp_socket_ptr_, inbuf_, boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
    io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    is_connected_ = true;
  }
  catch (std::exception& e)
  {
    printf("Exception: %s", e.what());
    is_connected_ = false;
    return false;
  }
  return true;
}

int16_t LaserDataReceiver::FindPacketStart()
{
  if(ring_buffer_.size() < kXSPackageHeadSize)
    return -1;

  // "HISN"
  for(size_t i = 0; i < ring_buffer_.size() - 4; i++){
    if(0x48 == ((unsigned char)ring_buffer_[i])   &&
       0x49 == ((unsigned char)ring_buffer_[i+1]) &&
       0x53 == ((unsigned char)ring_buffer_[i+2]) &&
       0x4e == ((unsigned char)ring_buffer_[i+3])){
      return i;
    }
  }
  return -2;
}

void LaserDataReceiver::WriteBufferBack(char *src, std::size_t num_bytes)
{
  if(ring_buffer_.size() + num_bytes > ring_buffer_.capacity()){
    throw std::exception();
  }


  ring_buffer_.resize(ring_buffer_.size() + num_bytes);

  char* pone = ring_buffer_.array_one().first;                    // const pointer
  std::size_t pone_size = ring_buffer_.array_one().second;        // size_type
  char* ptwo = ring_buffer_.array_two().first;
  std::size_t ptwo_size = ring_buffer_.array_two().second;

  if(ptwo_size >= num_bytes) {
    std::memcpy(ptwo + ptwo_size - num_bytes, src, num_bytes);
  } else {
    std::memcpy(pone + pone_size + ptwo_size - num_bytes, src, num_bytes - ptwo_size);
    std::memcpy(ptwo, src + num_bytes - ptwo_size, ptwo_size);
  }
}

bool LaserDataReceiver::HandleNextPacket()
{
  if(scan_data_.empty()){
    scan_data_.emplace_back();
  }

  if(!RetrivePacket()){
    return false;
  }else{
    return true;
  }
}

bool LaserDataReceiver::RetrivePacket()
{
  bool ret = false;
  int16_t head_index = FindPacketStart();
  if(head_index < 0) {
    return ret;
  }

  // get package header
  if(ring_buffer_.size() - head_index >= kXSPackageHeadSize)
  {
    ring_buffer_.erase_begin(head_index);
    head_index = 0;

    // 1. parse data header
    char head_buf[kXSPackageHeadSize];
    ReadBufferFront(head_buf, kXSPackageHeadSize);

    XSPackageHeader header;
    header.start_angle = (unsigned char)head_buf[4] << 8;     // 起始角度
    header.start_angle |= (unsigned char)head_buf[5];
    header.end_angle = (unsigned char)head_buf[6] << 8;       // 终止角度
    header.end_angle |= (unsigned char)head_buf[7];
    header.data_size = (unsigned char)head_buf[8] << 8;       // 测量点总数
    header.data_size |= (unsigned char)head_buf[9];
    header.data_position = (unsigned char)head_buf[10] << 8;  // 当前帧测量点位置
    header.data_position |= (unsigned char)head_buf[11];
    header.measure_size = (unsigned char)head_buf[12] << 8;   // 当前帧测量点数量
    header.measure_size |= (unsigned char)head_buf[13];

    //LOG_INFO("data size: %d, measure size: %d", header.data_size, header.measure_size);
    if(header.data_size > header.measure_size)
    {
      header.data_size = header.measure_size;
    }

    std::unique_lock<std::mutex> lock(scan_mutex_);

    // 2. parse data body
    ScanData& scan_data = scan_data_.back();
    uint16_t body_size = kXSPackageHeadSize + header.data_size * 4;
    if((ring_buffer_.size() - head_index) >= body_size)
    {
      // read data
      char* body_buf = new char[body_size];
      ReadBufferFront(body_buf, body_size);
      // deleta data
      ring_buffer_.erase_begin(head_index + body_size);

      for(int i = 0; i < header.data_size; i++)
      {
        unsigned short int distance;
        unsigned short int intensity;
        distance = (unsigned char)body_buf[i*4 + 17] * 256;
        distance |= (unsigned char)body_buf[i*4 + 16];
        intensity = (unsigned char)body_buf[i*4 + 19] * 256;
        intensity |= (unsigned char)body_buf[i*4 + 18];

        if(distance > kMaxDistance){
          distance = kMaxDistance;
        }
        if(intensity > kMaxIntensity){
          intensity = kMaxIntensity;
        }

        scan_data.distance_data.push_back(distance);
        scan_data.amplitude_data.push_back(intensity);

        // @todo: point number check
      }

      delete [] body_buf;

      // receive whole package
      if(header.end_angle == 0 && header.data_position == header.measure_size){
        scan_data_.emplace_back();
        if(scan_data_.size() > 5){
          scan_data_.pop_front();
        }
        data_notifier_.notify_one();
        last_data_time_ = hins::Now();
        ret = true;
      }

      if(FindPacketStart() >= 0){
        ret = true;
      }
    }
  }

  return ret;
}

void LaserDataReceiver::ReadBufferFront(char *dst, const uint16_t &num_bytes)
{
  if( ring_buffer_.size() < num_bytes )
    throw std::exception();

  char* pone = ring_buffer_.array_one().first;
  std::size_t pone_size = ring_buffer_.array_one().second;
  char* ptwo = ring_buffer_.array_two().first;

  if( pone_size >= num_bytes )
  {
    std::memcpy(dst, pone, num_bytes);
  }
  else
  {
    std::memcpy(dst, pone, pone_size);
    std::memcpy(dst + pone_size, ptwo, num_bytes - pone_size);
  }
}

}
