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
#include <iostream>
#include <unordered_map>

namespace odrive_ros_control
{
namespace transport
{
class SerialDevice
{
public:
  SerialDevice(unsigned int baud, const std::string& device) : ok_(true)
  {
    out_buffer_.reserve(static_cast<int>(AxisNumber::NUM_AXES));
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

  void load_buffer(std::string&& msg)
  {
    out_buffer_.emplace_back(msg);
  }

  void write_buffer()
  {
    write_async(std::accumulate(out_buffer_.begin(), out_buffer_.end(), std::string()));
    out_buffer_.clear();
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
                                  boost::bind(&SerialDevice::read_async_complete, this,
                                              boost::asio::placeholders::error,
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
      ROS_DEBUG_STREAM("write_async error");
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
  std::vector<std::string> out_buffer_;
};

class UartTransport : public CommandTransport
{
  using CommandTransport::init_transport;
  static const int default_baud = 921600;

public:
  bool init_transport(ros::NodeHandle& nh, std::string param_namespace, std::vector<std::string>& joint_names)
  {
    CommandTransport::init_transport(nh, param_namespace, joint_names);
    // check each joint's config on param server
    // skip if not defined
    // if it is defined it must have port and axis number, also if serial fails to open program terminates
    int baud;
    std::string port;
    int axis_number;
    std::string param_path = param_namepsace_ + param_prefix + "uart/joint_mapping/";
    for (auto joint_name : joint_names_)
    {
      ROS_DEBUG_STREAM("initializing joint:" + joint_name);
      if (!nh_ptr_->getParam(param_path + joint_name + "/port", port))
      {
        continue;
      }
      if (!nh_ptr_->getParam(param_path + joint_name + "/axis_number", axis_number))
      {
        continue;
      }
      if (axis_number >= static_cast<int>(AxisNumber::NUM_AXES))
      {
        continue;
      }
      if (!nh_ptr_->getParam(param_path + joint_name + "/baud", baud))
      {
        baud = default_baud;
      }
      if (serial_mapping_.find(port) == serial_mapping_.end())
      {
        serial_mapping_[port] = std::make_shared<SerialDevice>(baud, port);
      }
      JointConfig config{ port, static_cast<AxisNumber>(axis_number), serial_mapping_[port] };
      config_mapping_[joint_name] = config;
      ROS_DEBUG_STREAM("initialized joint:" + joint_name);
    }
  }

  bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd)
  {
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (config_defined(joint_names_[i]))
      {
        config_mapping_[joint_names_[i]].serial_device->load_buffer(
            boost::str(pos_cmd_fmt_ % static_cast<int>(config_mapping_[joint_names_[i]].axis) %
                       truncate_decimals(position_cmd[i]) % truncate_decimals(velocity_cmd[i])));
        config_mapping_[joint_names_[i]].serial_device->write_buffer();
      }
    }
  }

  bool receive(std::vector<double>& position)
  {
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (config_defined(joint_names_[i]))
      {
        config_mapping_[joint_names_[i]].serial_device->write_async(
            "f" + std::to_string(static_cast<int>(config_mapping_[joint_names_[i]].axis)) + "\n");
      }
    }
  }

private:
  struct JointConfig
  {
    std::string port;
    AxisNumber axis;
    std::shared_ptr<SerialDevice> serial_device;
  };
  std::unordered_map<std::string, std::shared_ptr<SerialDevice>> serial_mapping_;
  std::unordered_map<std::string, JointConfig> config_mapping_;
  std::unique_ptr<SerialDevice> serial_device_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  boost::basic_format<char> pos_cmd_fmt_ = boost::format("p %1% %2$.3f %3$.3f 0\n");

  double truncate_decimals(double& x)
  {
    return std::abs(x) > 0.001 ? x : 0;
  }

  bool config_defined(std::string& joint_name)
  {
    return config_mapping_.find(joint_name) != config_mapping_.end();
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::UartTransport, odrive_ros_control::transport::CommandTransport)