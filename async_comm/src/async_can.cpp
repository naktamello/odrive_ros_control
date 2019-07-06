#include <async_comm/async_can.h>


namespace async_comm
{
CanDevice::CanDevice(std::string device_name, CanRxCallback cb) : device_name_(device_name), callback_(cb)
{
  io_service_ = std::make_shared<boost::asio::io_service>();
  struct sockaddr_can addr
  {
  };
  struct ifreq ifr
  {
  };
  int result;
  can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ == -1)
    setup_error("error opening CAN_RAW socket");
  int enable = 1;
  result = ::setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable));
  if (result == -1)
    setup_error("error in setsockopt");
  std::strncpy(ifr.ifr_name, device_name_.c_str(), IFNAMSIZ);
  if (::ioctl(can_socket_, SIOCGIFINDEX, &ifr) == -1)
    setup_error("error in ioctl");
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  result = ::bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
  if (result == -1)
    setup_error("error while binding to network interface");
  socket_ = std::make_shared<boost::asio::posix::stream_descriptor>(*io_service_, can_socket_);
  boost::thread t(boost::bind(&CanDevice::start_thread, this));
}
CanDevice::~CanDevice()
{
  if (can_socket_ != -1)
    ::close(can_socket_);
  io_service_->stop();
}

void CanDevice::write_async(CanFrame &frame)
{
  boost::asio::async_write(*socket_, boost::asio::buffer(&frame, sizeof(frame)),
                           boost::bind(&CanDevice::write_async_complete, this, boost::asio::placeholders::error));
}

void CanDevice::start_thread()
{
  start_reading();
  io_service_->run();
}

void CanDevice::start_reading()
{
  read(can_socket_, read_buffer_, read_length);
  std::memcpy(&frame_, read_buffer_, read_length);
  callback_(frame_);
  boost::asio::async_read(*socket_, boost::asio::buffer(read_buffer_, read_length),
                          boost::bind(&CanDevice::read_async_complete, this, boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));
}

void CanDevice::read_async_complete(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (!error)
  {
    std::memcpy(&frame_, read_buffer_, bytes_transferred);
    callback_(frame_);
    // print_can_msg(frame_);
  }
  else
  {
    std::cout << "read error!" << std::endl;
  }
  start_reading();
}

void CanDevice::write_async_complete(const boost::system::error_code &error)
{
  if (!error)
  {
    // std::cout << "write async complete" << std::endl;
  }
  else
  {
    std::cout << "write error!" + error.message() << std::endl;
  }
}

void CanDevice::print_can_msg(CanFrame &frame)
{
  std::stringstream ss;
  ss << std::hex << frame.can_id << "(" << static_cast<int>(frame.can_dlc) << "):";
  for (std::size_t i = 0; i < frame.can_dlc; ++i)
  {
    if (i > 0)
      ss << " ";
    ss << std::hex << static_cast<int>(frame.data[i]);
  }
}

void CanDevice::setup_error(const std::string &msg)
{
  if (can_socket_ != -1)
    ::close(can_socket_);
}
}
