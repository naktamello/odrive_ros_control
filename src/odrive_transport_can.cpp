/*
 * Author: naktamello
 */
// stl
#include <iostream>
#include <sstream>
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
class CanDevice
{
};

class CanTransport : public CommandTransport
{
  using CommandTransport::init_transport;

public:
  bool init_transport(ros::NodeHandle& nh, std::string param_namespace, std::vector<std::string>& joint_names)
  {
    CommandTransport::init_transport(nh, param_namespace, joint_names);
    ROS_DEBUG_STREAM("CanTransport::init_transport()");
  }

  bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd)
  {
  }
  bool receive(std::vector<double>& position, std::vector<double>& velocity)
  {
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::CanTransport, odrive_ros_control::transport::CommandTransport)