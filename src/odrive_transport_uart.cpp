/*
 * Author: naktamello
 */
// stl
#include <iostream>
#include <sstream>
#include <unordered_map>
// boost
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
// odrive_ros_control
#include <odrive_ros_control/transport_interface.h>

namespace odrive_ros_control
{
namespace transport
{
class SerialDevice
{
public:
  using RequestCallback = std::function<void(std::string)>;
  SerialDevice(unsigned int baud, const std::string& device) : ok_(true)
  {
    out_buffer_.reserve(static_cast<int>(AxisNumber::NUM_AXES));
    io_service_ = std::make_shared<boost::asio::io_service>();
    serial_port_ = std::make_unique<boost::asio::serial_port>(*io_service_, device);
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

  void request_async(std::string&& msg, RequestCallback cb)
  {
    cb_service_.cb = cb;
    write_async(std::forward<std::string>(msg));
    cb_service_.state = RequestState::WAITING;
  }

  bool ok()
  {
    return ok_;
  }

  bool can_take_request()
  {
    return cb_service_.state == RequestState::IDLE;
  }

  int loop_count()
  {
    return cb_service_.wait_count++;
  }

  void reset_request_state()
  {
    cb_service_.state = RequestState::IDLE;
    cb_service_.wait_count = 0;
    cb_service_.cb = nullptr;
    start_reading();
  }

private:
  enum class RequestState
  {
    IDLE = 0,
    WAITING = 1
  };
  struct CallbackService
  {
    int wait_count;
    RequestState state;
    RequestCallback cb;
  };
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
        {
          if (io_buffer_[i] != '\r')
            read_buffer_.push_back(io_buffer_[i]);
        }
        else
        {
          std::string payload = std::string(std::accumulate(read_buffer_.begin(), read_buffer_.end(), std::string()));
          read_buffer_.clear();
          if (cb_service_.cb)
          {
            cb_service_.cb(payload);
            reset_request_state();
          }
        }
      }
      start_reading();
    }
    else
    {
      ROS_DEBUG_STREAM("read_async error");
      terminate(error);
    }
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

  // constants
  static const int max_read_length = 512;
  // pointers
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  // state
  bool ok_;
  CallbackService cb_service_ = { 0, RequestState::IDLE, nullptr };
  // buffers
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
    // if it is defined it must have port and axis number
    // if serial fails to open program terminates
    int baud;
    std::string port;
    int axis_number;
    std::string param_path = param_namepsace_ + param_prefix + "uart/joint_mapping/";
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto joint_name = joint_names_[i];
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
        serial_mapping_[port] = { std::make_shared<SerialDevice>(baud, port), std::vector<AxisNumber>(),
                                  AxisNumber::NONE };
      }
      serial_mapping_[port].active_axes.push_back(static_cast<AxisNumber>(axis_number));
      JointConfig config{
        static_cast<int>(i), port, static_cast<AxisNumber>(axis_number), &serial_mapping_[port], { 0, 0 }
      };
      config_mapping_[joint_name] = config;
      ROS_DEBUG_STREAM("initialized joint:" + joint_name);
    }
    for (auto& kv : serial_mapping_)
    {
      kv.second.current_axis = kv.second.active_axes[0];
    }
  }

  bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd)
  {
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (config_defined(joint_names_[i]))
      {
        JointConfig* config = &config_mapping_[joint_names_[i]];
        // ROS_DEBUG_STREAM("loading:" +
        //                  boost::str(pos_cmd_fmt_ % static_cast<int>(config_mapping_[joint_names_[i]].axis) %
        //                             truncate_small(position_cmd[i]) % truncate_small(velocity_cmd[i])));
        config->serial_object->device->load_buffer(
            boost::str(pos_cmd_fmt_ % static_cast<int>(config_mapping_[joint_names_[i]].axis) %
                       truncate_small(position_cmd[i]) % truncate_small(velocity_cmd[i])));
        config->serial_object->device->write_buffer();
      }
    }
  }

  bool receive(std::vector<double>& position, std::vector<double>& velocity)
  {
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (config_defined(joint_names_[i]))
      {
        JointConfig* config = &config_mapping_[joint_names_[i]];
        if ((config->serial_object->current_axis == config->axis) && config->serial_object->device->can_take_request())
        {
          PosVel pos_vel = { &position[i], &velocity[i] };
          config->serial_object->device->request_async(
              "f " + std::to_string(static_cast<int>(config->axis)) + "\n",
              std::bind(&UartTransport::feedback_request_callback, this, config, pos_vel, std::placeholders::_1));
        }
        if (config->serial_object->device->loop_count() > max_wait)
        {
          config->serial_object->device->reset_request_state();
          ROS_DEBUG_STREAM("no response, resetting request");
        }
      }
    }
  }

private:
  struct SerialObject
  {
    std::shared_ptr<SerialDevice> device;
    std::vector<AxisNumber> active_axes;
    AxisNumber current_axis;
  };
  struct JointConfig
  {
    int joint_idx;
    std::string port;
    AxisNumber axis;
    SerialObject* serial_object;
    std::array<double, 2> pos_vel;
  };
  using PosVel = std::array<double*, 2>;

  std::unordered_map<std::string, SerialObject> serial_mapping_;
  std::unordered_map<std::string, JointConfig> config_mapping_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  boost::basic_format<char> pos_cmd_fmt_ = boost::format("p %1% %2$.3f %3$.3f 0\n");

  static const int max_wait = 10;

  double truncate_small(double& x)
  {
    return std::abs(x) > 0.001 ? x : 0;
  }

  bool config_defined(std::string& joint_name)
  {
    return config_mapping_.find(joint_name) != config_mapping_.end();
  }
  void queue_next_axis(SerialObject* serial_object)
  {
    int i = 0;
    for (i; i < serial_object->active_axes.size(); ++i)
    {
      if (serial_object->current_axis == serial_object->active_axes[i])
        break;
    }
    i = (i + 1) % serial_object->active_axes.size();
    serial_object->current_axis = static_cast<AxisNumber>(i);
  }

  void feedback_request_callback(JointConfig* config, PosVel pos_vel, std::string payload)
  {
    std::string axis = std::to_string(static_cast<int>(config->axis));
    queue_next_axis(config->serial_object);
    parse_joint_feedback(pos_vel, payload);
  }

  bool parse_joint_feedback(PosVel& pos_vel, std::string& payload)
  {
    try
    {
      std::string item;
      std::stringstream ss(payload);
      int i = 0;
      while (std::getline(ss, item, ' '))
      {
        if (i == pos_vel.size())
        {
          ROS_DEBUG_STREAM("parse_joint_feedback:out of bounds");
          return false;
        }
        *pos_vel[i++] = boost::lexical_cast<double>(item);
      }
      return true;
    }
    catch (boost::bad_lexical_cast)
    {
      ROS_DEBUG_STREAM("parse_joint_feedback:bad cast");
      return false;
    }
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::UartTransport, odrive_ros_control::transport::CommandTransport)