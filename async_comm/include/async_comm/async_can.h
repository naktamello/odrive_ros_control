#include <iostream>
// linux
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
// boost
#include <boost/any.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>


namespace async_comm
{
using CanFrame = struct can_frame;
using CanRxCallback = std::function<void(CanFrame)>;
class CanDevice
{
public:
  CanDevice(std::string device_name, CanRxCallback cb);
  ~CanDevice();
  void write_async(CanFrame &frame);

private:
  void start_thread();
  void start_reading();
  void read_async_complete(const boost::system::error_code &error, size_t bytes_transferred);
  void write_async_complete(const boost::system::error_code &error);
  void print_can_msg(CanFrame &frame);
  void setup_error(const std::string &msg);

  int can_socket_;
  std::string device_name_;
  static const size_t read_length = sizeof(CanFrame);
  char read_buffer_[read_length] = { 0 };
  CanFrame frame_{};
  std::shared_ptr<boost::asio::posix::stream_descriptor> socket_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  CanRxCallback callback_;
};
}
