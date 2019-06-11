/*
 * Author: naktamello
 */
// stl
#include <iostream>
#include <sstream>
// linux
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
// boost
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
// odrive_ros_control
#include <odrive_ros_control/transport_interface.h>
namespace odrive_ros_control
{
namespace transport
{
using CanFrame = struct can_frame;
class CanDevice
{
public:
  CanDevice(std::string device_name) : device_name_(device_name)
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
    result = ::bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (result == -1)
      setup_error("error while binding to network interface");
    socket_ = std::make_unique<boost::asio::posix::stream_descriptor>(*io_service_, can_socket_);
    boost::thread t(boost::bind(&boost::asio::io_service::run, io_service_));
    start_reading();
  }
  ~CanDevice()
  {
    if (can_socket_ != -1)
      ::close(can_socket_);
  }
  void write_async(CanFrame& frame)
  {
    boost::asio::async_write(*socket_, boost::asio::buffer(&frame, sizeof(frame)),
                             boost::bind(&CanDevice::write_async_complete, this, boost::asio::placeholders::error));
  }

private:
  void start_reading()
  {
    boost::asio::async_read(*socket_, boost::asio::buffer(read_buffer_, read_length),
                            boost::bind(&CanDevice::read_async_complete, this, boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
  }
  void read_async_complete(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (!error)
    {
      std::cout << "received:" + std::to_string(bytes_transferred) << std::endl;
      std::memcpy(&frame_, read_buffer_, bytes_transferred);
      write_async(frame_);
      print_can_msg(frame_);
      start_reading();
    }
    else
    {
      std::cout << "read error!" << std::endl;
    }
  }

  void write_async_complete(const boost::system::error_code& error)
  {
    if (!error)
    {
      std::cout << "write async complete" << std::endl;
    }
    else
    {
      std::cout << "write error!" << std::endl;
    }
  }

  void print_can_msg(CanFrame& frame)
  {
    std::stringstream ss;
    ss << std::hex << frame.can_id << "(" << static_cast<int>(frame.can_dlc) << "):";
    for (std::size_t i = 0; i < frame.can_dlc; ++i)
    {
      if (i > 0)
        ss << " ";
      ss << std::hex << static_cast<int>(frame.data[i]);
    }
    ROS_DEBUG_STREAM(ss.str());
  }

  void setup_error(const std::string& msg)
  {
    ROS_FATAL_STREAM(msg);
    if (can_socket_ != -1)
      ::close(can_socket_);
    ros::shutdown();
  }

  int can_socket_;
  std::string device_name_;
  static const size_t read_length = sizeof(CanFrame);
  char read_buffer_[read_length] = { 0 };
  CanFrame frame_{};
  std::unique_ptr<boost::asio::posix::stream_descriptor> socket_;
  std::shared_ptr<boost::asio::io_service> io_service_;
};

class CanTransport : public CommandTransport
{
  using CommandTransport::init_transport;

public:
  bool init_transport(ros::NodeHandle& nh, std::string param_namespace, std::vector<std::string>& joint_names)
  {
    CommandTransport::init_transport(nh, param_namespace, joint_names);
    ROS_DEBUG_STREAM("CanTransport::init_transport()");
    std::string param_path = param_namepsace_ + param_prefix + "can";
    std::string device_name;
    if (!nh_ptr_->getParam(param_path + "/device_name", device_name))
    {
      ROS_FATAL_STREAM("you must provide can device name--i.e. 'slcan0'--in the param server!");
      ros::shutdown();
    }
    can_device_ = std::make_unique<CanDevice>(device_name);
  }

  bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd)
  {
  }
  bool receive(std::vector<double>& position, std::vector<double>& velocity)
  {
  }

private:
  std::unique_ptr<CanDevice> can_device_;
};
}
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::CanTransport, odrive_ros_control::transport::CommandTransport)