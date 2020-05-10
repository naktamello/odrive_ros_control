/*
 * Author: naktamello
 */
// stl
#include <iostream>
#include <sstream>
// boost
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>
// odrive_ros_control
#include <odrive_ros_control/transport_interface.h>
// async_comm
#include <async_comm/async_serial.h>
using namespace async_comm;

namespace odrive_ros_control
{
namespace transport
{
class UartTransport : public CommandTransport
{
  using CommandTransport::init_transport;
  static const int default_baud = 921600;

public:
  bool init_transport(std::shared_ptr<ros::NodeHandle> nh, std::string param_namespace,
                      std::vector<std::string>& joint_names)
  {
    CommandTransport::init_transport(nh, param_namespace, joint_names);
    // check each joint's config on param server
    // skip if not defined
    // if it is defined it must have port and axis number
    // if serial fails to open program terminates
    int baud;
    std::string port;
    int axis_number;
    auto joint_param_path = param_path_ + "joint_mapping/";
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto joint_name = joint_names_[i];
      ROS_DEBUG_STREAM("initializing joint:" + joint_name);
      if (!nh_ptr_->getParam(joint_param_path + joint_name + "/port", port))
      {
        continue;
      }
      if (!nh_ptr_->getParam(joint_param_path + joint_name + "/axis_number", axis_number))
      {
        continue;
      }
      if (axis_number >= static_cast<int>(AxisNumber::NUM_AXES))
      {
        continue;
      }
      if (!nh_ptr_->getParam(joint_param_path + joint_name + "/baud", baud))
      {
        baud = default_baud;
      }
      if (serial_mapping_.find(port) == serial_mapping_.end())
      {
        serial_mapping_[port] = { std::make_shared<SerialDevice>(baud, port, static_cast<int>(AxisNumber::NUM_AXES)),
                                  std::vector<AxisNumber>(), AxisNumber::NONE };
      }
      serial_mapping_[port].active_axes.push_back(static_cast<AxisNumber>(axis_number));
      config_mapping_[joint_name] = UartJointConfig(static_cast<int>(i), { 0, 0 }, port,
                                                    static_cast<AxisNumber>(axis_number), &serial_mapping_[port]);
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
      if (config_defined(config_mapping_, joint_names_[i]))
      {
        UartJointConfig* config = boost::any_cast<UartJointConfig>(&config_mapping_[joint_names_[i]]);
        // ROS_DEBUG_STREAM("loading:" +
        //                  boost::str(pos_cmd_fmt_ % static_cast<int>(config_mapping_[joint_names_[i]].axis) %
        //                             truncate_small(position_cmd[i]) % truncate_small(velocity_cmd[i])));
        config->serial_object->device->load_buffer(
            boost::str(pos_cmd_fmt_ % static_cast<int>(config->axis) % truncate_small(position_cmd[i]) %
                       truncate_small(velocity_cmd[i])));
        config->serial_object->device->write_buffer();
      }
    }
    return true;
  }
  bool send(std::vector<double>& velocity_cmd)
  {
    ROS_WARN_STREAM("velocity interface is not yet supported with UART!");
  }
  bool receive(std::vector<double>& position, std::vector<double>& velocity)
  {
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (config_defined(config_mapping_, joint_names_[i]))
      {
        UartJointConfig* config = boost::any_cast<UartJointConfig>(&config_mapping_[joint_names_[i]]);
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
    return true;
  }

private:
  struct SerialObject
  {
    std::shared_ptr<SerialDevice> device;
    std::vector<AxisNumber> active_axes;
    AxisNumber current_axis;
  };
  struct UartJointConfig : JointConfig
  {
    UartJointConfig(){};
    UartJointConfig(int joint_idx_, std::array<double, 2> pos_vel_, std::string port_, AxisNumber axis_,
                    SerialObject* serial_object_)
      : JointConfig(joint_idx_, pos_vel_), port(port_), axis(axis_), serial_object(serial_object_)
    {
    }
    std::string port;
    AxisNumber axis;
    SerialObject* serial_object;
  };
  using PosVel = std::array<double*, 2>;

  boost::unordered_map<std::string, SerialObject> serial_mapping_;
  ConfigMapping config_mapping_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  boost::basic_format<char> pos_cmd_fmt_ = boost::format("p %1% %2$.3f %3$.3f 0\n");

  static const int max_wait = 10;

  double truncate_small(double& x)
  {
    return std::abs(x) > 0.001 ? x : 0;
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

  void feedback_request_callback(UartJointConfig* config, PosVel pos_vel, std::string payload)
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
}  // namespace transport
}  // namespace odrive_ros_control

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::UartTransport, odrive_ros_control::transport::CommandTransport)