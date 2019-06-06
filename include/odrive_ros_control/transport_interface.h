/*
 * Author: naktamello
 */
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace odrive_ros_control
{
  enum class AxisNumber
  {
    NONE = -1,
    AXIS0 = 0,
    AXIS1 = 1,
    NUM_AXES
  };
namespace transport
{
enum TransportType
{
  UART = 0,
  CAN = 1,
  USB_NATIVE = 2
};
const std::string param_prefix = "/odrive/transport/";
class CommandTransport
{
public:
  //   CommandTransport(std::string param_namespace):
  virtual bool init_transport(ros::NodeHandle& nh, std::string param_namespace, std::vector<std::string>& joint_names)
  { 
    nh_ptr_ = std::make_shared<ros::NodeHandle>(nh);
    param_namepsace_ = param_namespace;
    joint_names_ = joint_names;
    position_.resize(joint_names_.size());
  };
  virtual ~CommandTransport()
  {
  }
  virtual bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd) = 0;
  virtual bool receive(std::vector<double>& position, std::vector<double>& velocity) = 0;

protected:
  std::shared_ptr<ros::NodeHandle> nh_ptr_;
  std::string param_namepsace_;
  std::vector<std::string> joint_names_;
  std::vector<double> position_;
  std::vector<double> velocity_;
};
}
}