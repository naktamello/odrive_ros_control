/*
 * Author: naktamello
 */
#include <odrive_ros_control/transport_interface.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <unordered_map>
#include <iostream>

namespace odrive_ros_control
{
namespace transport
{
/*
 * SerialDevice class is adaptation of async serial port example by Jeff Gray(2008).
 * http://boost.2283326.n4.nabble.com/Simple-serial-port-demonstration-with-boost-asio-asynchronous-I-O-td2582657.html
 */
class SerialDevice
{
public:
  // SerialDevice(boost::asio::io_service& io_service, unsigned int baud, const std::string& device)
    // : ok_(true), io_service_(io_service), serial_port_(io_service, device)
  SerialDevice(unsigned int baud, const std::string& device)
    : ok_(true)
  {
    io_service_ = std::make_shared<boost::asio::io_service>();
    serial_port_ = std::unique_ptr<boost::asio::serial_port>(new boost::asio::serial_port(*io_service_, device));
    if (!serial_port_->is_open())
    {
      ROS_FATAL_NAMED("odrive_ros_control", "Cannot open serial port!");
      ros::shutdown();
    }
    boost::asio::serial_port_base::baud_rate baud_option(baud);
    serial_port_->set_option(baud_option);
    boost::thread t(boost::bind(&boost::asio::io_service::run, io_service_));
    start_reading();
  }

  void write_async(std::string&& msg)
  {
    boost::asio::async_write(*serial_port_, boost::asio::buffer(msg),
                             boost::bind(&SerialDevice::write_async_complete, this, boost::asio::placeholders::error));
  }

  bool ok()
  {
    return ok_;
  }

private:

  void start_reading(void)
  {
    serial_port_->async_read_some(boost::asio::buffer(io_buffer_, max_read_length),
                                 boost::bind(&SerialDevice::read_async_complete, this, boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));
  }

  void read_async_complete(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (!error)
    {
      for (auto i = 0; i < bytes_transferred; ++i)
      {
        if (io_buffer_[i] != '\n')
          read_buffer_.push_back(io_buffer_[i]);
        else
        {
          std::string payload = std::string(std::accumulate(read_buffer_.begin(), read_buffer_.end(), std::string()));
          ROS_DEBUG_STREAM("read complete:" << payload);
          read_buffer_.clear();
        }
      }
      start_reading();
    }
    else
      terminate(error);
  }

  void write_async_complete(const boost::system::error_code& error)
  {
    if (!error)
    {
    }
    else
    {
      terminate(error);
    }
    // ROS_DEBUG_STREAM("write_async_complete");
  }

  void terminate(const boost::system::error_code& error)
  { 
    if (error == boost::asio::error::operation_aborted)
      return;
    ROS_FATAL_NAMED("odrive_ros_control", (std::string("could not open serial port: ") + error.message()).c_str());
    serial_port_->close();
    ok_ = false;
    ros::shutdown();
  }

  std::unique_ptr<boost::asio::serial_port> serial_port_;
  static const int max_read_length = 96;
  bool ok_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  char io_buffer_[max_read_length];
  boost::circular_buffer<char> read_buffer_{ max_read_length };
};

class UartTransport : public CommandTransport
{
  using CommandTransport::init_transport;
  static const int default_baud = 115200;

public:
  bool init_transport(ros::NodeHandle& nh, std::string param_namespace, std::vector<std::string>& joint_names)
  {
    CommandTransport::init_transport(nh, param_namespace, joint_names);
    // check each joint's config on param server
    // skip if not defined
    // if it is defined it must have port and axis number, also if serial fails to open program terminates
    int baud;
    if (!nh_ptr_->getParam(param_namepsace_ + param_prefix + "uart/baud", baud))
    {
      baud = default_baud;
      ROS_DEBUG_STREAM_NAMED("UartTransport", "using default baudrate");
    }
    std::string port;
    if (!nh_ptr_->getParam(param_namepsace_ + param_prefix + "uart/port", port))
    {
      ROS_FATAL_STREAM_NAMED("UartTransport", "you must specify uart port on param server!"
                                                  << param_namepsace_ + param_prefix + "uart/port");
      ros::shutdown();
    }
    serial_device_ = std::unique_ptr<SerialDevice>(new SerialDevice(baud, port));
    
  }

  bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd)
  {
    // TODO map joint to serial port
    serial_device_->write_async(boost::str(pos_cmd_fmt_ % 0 % position_cmd[0] % velocity_cmd[0]));

  }

  bool receive(std::vector<double>& position)
  {
    serial_device_->write_async(std::string("f 0\n"));

  }

private:
  struct JointConfig{
    std::string port;
    AxisNumber axis;
    
  };
  std::unordered_map<std::string, std::shared_ptr<SerialDevice>> serial_mapping_;
  std::unordered_map<std::string, std::string> config_mapping_;
  std::unique_ptr<SerialDevice> serial_device_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  boost::basic_format<char> pos_cmd_fmt_ = boost::format("p %1% %2$.4f %3$.4f 0\n");
};
}
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::UartTransport, odrive_ros_control::transport::CommandTransport)