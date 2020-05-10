/*
 * Author: naktamello
 */
// ros
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
// stl
#include <string>
#include <vector>
#include <array>
// boost
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>
#include <boost/type_index.hpp>
#include <boost/unordered_map.hpp>
// odrive_ros_conrol
#include <odrive_ros_control/SetReadMode.h>
#include <odrive_ros_control/SetWriteMode.h>

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

struct JointState
{
  bool initialized;
  double position;
  double velocity;
  uint32_t axis_error;
  uint32_t current_state;
};

const std::string param_prefix = "/odrive/transport/";
class CommandTransport
{
public:
  virtual bool init_transport(std::shared_ptr<ros::NodeHandle> nh, std::string param_namespace, std::vector<std::string>& joint_names)
  {
    read_on_ = false;
    write_on_ = false;
    nh_ptr_ = nh;
    nh_ptr_->getParam(param_prefix + "interface", transport_type_);
    std::transform(transport_type_.begin(), transport_type_.end(), transport_type_.begin(), ::tolower);
    param_namespace_ = param_namespace;
    param_path_ = param_namespace_ + param_prefix + transport_type_ + "/";
    joint_names_ = joint_names;
    position_.resize(joint_names_.size());

    services_.push_back(
        nh_ptr_->advertiseService("/odrive_ros_control/set_read_mode", &CommandTransport::handle_set_read_mode, this));
    services_.push_back(
        nh_ptr_->advertiseService("/odrive_ros_control/set_write_mode", &CommandTransport::handle_set_write_mode, this));
  };
  virtual ~CommandTransport()
  {
  }
  virtual bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd) = 0;
  virtual bool send(std::vector<double>& velocity_cmd) = 0;
  virtual bool receive(std::vector<double>& position, std::vector<double>& velocity) = 0;

protected:
  std::shared_ptr<ros::NodeHandle> nh_ptr_;
  std::vector<ros::ServiceServer> services_;
  std::string param_namespace_;
  std::string param_path_;
  std::string transport_type_;
  std::vector<std::string> joint_names_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  bool read_on_;
  bool write_on_;
  virtual bool config_defined(const ConfigMapping& config_mapping, std::string& joint_name)
  {
    return config_mapping.find(joint_name) != config_mapping.end();
  }
  virtual bool handle_set_read_mode(odrive_ros_control::SetReadMode::Request& req,
                                    odrive_ros_control::SetReadMode::Response& res)
  {
    read_on_ = req.read_on;
    res.result = "success";
    ROS_DEBUG_STREAM("handle_set_read_mode:" + (req.read_on == true) ? "ON" : "OFF");
  }
  virtual bool handle_set_write_mode(odrive_ros_control::SetWriteMode::Request& req,
                                     odrive_ros_control::SetWriteMode::Response& res)
  {
    write_on_ = req.write_on;
    ROS_DEBUG_STREAM("handle_set_write_mode:" + (req.write_on == true) ? "ON" : "OFF");
  }
};
}
}