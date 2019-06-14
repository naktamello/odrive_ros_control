/*
 * Author: naktamello
 */
// ros
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
// stl
#include <string>
#include <vector>
// boost
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>
#include <boost/unordered_map.hpp>
#include <boost/type_index.hpp>

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
using ConfigMapping = boost::unordered_map<std::string, boost::any>;

enum TransportType
{
  UART = 0,
  CAN = 1,
  USB_NATIVE = 2
};

struct JointConfig
{
  JointConfig(){};
  JointConfig(int joint_idx_, std::array<double, 2> pos_vel_) : joint_idx(joint_idx_), pos_vel(pos_vel_)
  {
  }
  int joint_idx;
  std::array<double, 2> pos_vel;
};

const std::string param_prefix = "/odrive/transport/";
class CommandTransport
{
public:
  virtual bool init_transport(ros::NodeHandle& nh, std::string param_namespace, std::vector<std::string>& joint_names)
  {
    nh_ptr_ = std::make_shared<ros::NodeHandle>(nh);
    nh_ptr_->getParam(param_prefix + "interface", transport_type_);
    std::transform(transport_type_.begin(), transport_type_.end(), transport_type_.begin(), ::tolower);
    param_namespace_ = param_namespace;
    param_path_ = param_namespace_ + param_prefix + transport_type_ + "/";
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
  std::string param_namespace_;
  std::string param_path_;
  std::string transport_type_;
  std::vector<std::string> joint_names_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  virtual bool config_defined(const ConfigMapping& config_mapping, std::string& joint_name)
  {
    return config_mapping.find(joint_name) != config_mapping.end();
  }
};
}
}