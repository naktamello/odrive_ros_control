/*
 * Author: naktamello
 */
// stl
#include <iostream>
#include <mutex>
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
#include <boost/any.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
// odrive_ros_control
#include <odrive_ros_control/GetAxisError.h>
#include <odrive_ros_control/GetCurrentState.h>
#include <odrive_ros_control/GetVbusVoltage.h>
#include <odrive_ros_control/MoveToPos.h>
#include <odrive_ros_control/SetCurrentSetpoint.h>
#include <odrive_ros_control/SetPosSetpoint.h>
#include <odrive_ros_control/SetRequestedState.h>
#include <odrive_ros_control/SetVelSetpoint.h>
#include <odrive_ros_control/transport_interface.h>

namespace odrive_ros_control
{
namespace transport
{
namespace CanSimpleCommands
{
const int ODriveHeartbeatMessage = 0x001;
const int SetAxisRequestedState = 0x007;
const int GetEncoderEstimates = 0x009;
const int MoveToPos = 0x00B;
const int SetPosSetpoint = 0x00C;
const int SetVelSetpoint = 0x00D;
const int SetCurrentSetpoint = 0x00E;
const int GetVbusVoltage = 0x017;
}
enum class Endianness
{
  LITTLE = 0,
  BIG = 1
};
class CanSimpleSerializer
{
public:
  CanSimpleSerializer()
  {
    static_assert(std::numeric_limits<float>::is_iec559, "float type is not IEEE754");
    endian_ = byte_order();
  }

  template <typename T>
  void serialize(const T &value, uint8_t *dst)
  {
    auto src = reinterpret_cast<const uint8_t *>(&value);
    endian_copy(src, dst, sizeof(T));
  }

  template <typename T>
  T deserialize(uint8_t *src)
  {
    T dst;
    endian_copy(src, reinterpret_cast<uint8_t *>(&dst), sizeof(T));
    return dst;
  };

  void serialize_float(const float &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  float deserialize_float(uint8_t *src)
  {
    return deserialize<float>(src);
  }

  void serialize_uint32(const uint32_t &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  uint32_t deserialize_uint32(uint8_t *src)
  {
    return deserialize<uint32_t>(src);
  }

  void serialize_int32(const int32_t &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  int32_t deserialize_int32(uint8_t *src)
  {
    return deserialize<int32_t>(src);
  }

  void serialize_uint16(const uint16_t &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  uint16_t deserialize_uint16(uint8_t *src)
  {
    return deserialize<uint16_t>(src);
  }

  void serialize_int16(const int16_t &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  int16_t deserialize_int16(uint8_t *src)
  {
    return deserialize<int16_t>(src);
  }

  void endian_copy(const uint8_t *src, uint8_t *dst, size_t size)
  {
    if (endian_ == Endianness::LITTLE)
      std::memcpy(dst, src, size);
    else
    {
      size_t offset = size - 1;
      for (std::size_t i = 0; i < size; ++i)
      {
        *dst++ = *(src + offset--);
      }
    }
  }

  Endianness byte_order()
  {
    short int word = 0x00001;
    char *b = (char *)&word;
    return (b[0] ? Endianness::LITTLE : Endianness::BIG);
  }

private:
  Endianness endian_;
};
using CanFrame = struct can_frame;
class CanDevice
{
public:
  using CanRxCallback = std::function<void(CanFrame)>;
  CanDevice(std::string device_name, CanRxCallback cb) : device_name_(device_name), callback_(cb)
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
    result = ::bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
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
    io_service_->stop();
  }
  void write_async(CanFrame &frame)
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
  void read_async_complete(const boost::system::error_code &error, size_t bytes_transferred)
  {
    if (!error)
    {
      std::memcpy(&frame_, read_buffer_, bytes_transferred);
      callback_(frame_);
      // print_can_msg(frame_);
      start_reading();
    }
    else
    {
      std::cout << "read error!" << std::endl;
    }
  }

  void write_async_complete(const boost::system::error_code &error)
  {
    if (!error)
    {
      // std::cout << "write async complete" << std::endl;
    }
    else
    {
      std::cout << "write error!" + error.message() << std::endl;
    }
  }

  void print_can_msg(CanFrame &frame)
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

  void setup_error(const std::string &msg)
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
  CanRxCallback callback_;
};

class CanTransport : public CommandTransport
{
  using CommandTransport::init_transport;

public:
  CanTransport() : serializer_(), loop_cnt_(0)
  {
  }

  bool init_transport(ros::NodeHandle &nh, std::string param_namespace, std::vector<std::string> &joint_names)
  {
    CommandTransport::init_transport(nh, param_namespace, joint_names);
    ROS_DEBUG_STREAM("CanTransport::init_transport()");
    joint_states_.resize(joint_names.size());
    for (auto state : joint_states_)
    {
      state.initialized = false;
    }
    auto joint_param_path = param_path_ + "joint_mapping/";
    std::string device_name;
    if (!nh_ptr_->getParam(param_path_ + "device_name", device_name))
    {
      ROS_FATAL_STREAM("you must provide can device name--i.e. 'slcan0'--in the param server!");
      ros::shutdown();
    }
    can_device_ = std::make_unique<CanDevice>(device_name,
                                              std::bind(&CanTransport::can_rx_callback, this, std::placeholders::_1));

    int node_id;
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto joint_name = joint_names_[i];
      ROS_DEBUG_STREAM("initializing joint:" + joint_name);
      if (!nh_ptr_->getParam(joint_param_path + joint_name + "/node_id", node_id))
      {
        continue;
      }
      ROS_DEBUG_STREAM("initialized joint:" + joint_name);
      config_mapping_[joint_name] = CanJointConfig(static_cast<int>(i), { 0, 0 }, node_id);
    }
    services_.push_back(nh_ptr_->advertiseService("/odrive_ros_control/set_requested_state",
                                                  &CanTransport::handle_set_requested_state, this));
    services_.push_back(nh_ptr_->advertiseService("/odrive_ros_control/get_current_state",
                                                  &CanTransport::handle_get_current_state, this));
    services_.push_back(
        nh_ptr_->advertiseService("/odrive_ros_control/get_axis_error", &CanTransport::handle_get_axis_error, this));
    services_.push_back(nh_ptr_->advertiseService("/odrive_ros_control/get_vbus_voltage",
                                                  &CanTransport::handle_get_vbus_voltage, this));
    services_.push_back(nh_ptr_->advertiseService("/odrive_ros_control/set_pos_setpoint",
                                                  &CanTransport::handle_set_pos_setpoint, this));
    services_.push_back(nh_ptr_->advertiseService("/odrive_ros_control/set_vel_setpoint",
                                                  &CanTransport::handle_set_vel_setpoint, this));
    services_.push_back(nh_ptr_->advertiseService("/odrive_ros_control/set_current_setpoint",
                                                  &CanTransport::handle_set_current_setpoint, this));
    services_.push_back(
        nh_ptr_->advertiseService("/odrive_ros_control/move_to_pos", &CanTransport::handle_move_to_pos, this));
  }

  bool send(std::vector<double> &position_cmd, std::vector<double> &velocity_cmd)
  {
    if (!write_on_)
      return true;
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (config_defined(config_mapping_, joint_names_[i]))
      {
        CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[joint_names_[i]]);
        CanFrame can_frame = make_position_command(config->node_id, static_cast<int32_t>(position_cmd[i]),
                                                   static_cast<int16_t>(velocity_cmd[i]));
        std::lock_guard<std::mutex> guard(can_mutex_);
        can_device_->write_async(can_frame);
      }
    }
    return true;
  }
  bool receive(std::vector<double> &position, std::vector<double> &velocity)
  {
    if (!read_on_)
      return true;
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (config_defined(config_mapping_, joint_names_[i]))
      {
        CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[joint_names_[i]]);
        if (joint_states_[i].initialized)
        {
          position[i] = joint_states_[i].position;
          velocity[i] = joint_states_[i].velocity;
        }
        CanFrame can_frame = make_feedback_command(config->node_id);
        std::lock_guard<std::mutex> guard(can_mutex_);
        can_device_->write_async(can_frame);
      }
    }
    return true;
  }

private:
  uint8_t srv_data_[8];
  bool srv_ready_;
  std::mutex can_mutex_;
  uint32_t loop_cnt_;
  std::unique_ptr<CanDevice> can_device_;
  CanSimpleSerializer serializer_;
  std::vector<JointState> joint_states_;
  struct CanJointConfig : JointConfig
  {
    CanJointConfig(){};
    CanJointConfig(int joint_idx_, std::array<double, 2> pos_vel_, int node_id_)
      : JointConfig(joint_idx_, pos_vel_), node_id(node_id_)
    {
    }
    int node_id;
  };
  ConfigMapping config_mapping_;
  CanFrame make_move_to_pos_command(int node_id, const int32_t &position)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::MoveToPos;
    serializer_.serialize_int32(position, can_frame.data);
    return can_frame;
  }
  CanFrame make_position_command(int node_id, const int32_t &position, const int16_t &velocity)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::SetPosSetpoint;
    serializer_.serialize_int32(position, can_frame.data);
    serializer_.serialize_int16(velocity, &can_frame.data[4]);
    return can_frame;
  }
  CanFrame make_velocity_command(int node_id, const int32_t &velocity, const int16_t &current)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::SetVelSetpoint;
    serializer_.serialize_int32(velocity, can_frame.data);
    serializer_.serialize_int16(current, &can_frame.data[4]);
    return can_frame;
  }
  CanFrame make_current_command(int node_id, const int32_t &current)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::SetCurrentSetpoint;
    serializer_.serialize_int32(current, can_frame.data);
    return can_frame;
  }
  CanFrame make_feedback_command(int node_id)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::GetEncoderEstimates | (1 << 30);
    return can_frame;
  }
  CanFrame make_vbus_command(int node_id)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::GetVbusVoltage | (1 << 30);
    return can_frame;
  }
  CanFrame make_set_requested_state_command(int node_id, uint8_t requested_state)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::SetAxisRequestedState;
    serializer_.serialize_int32(static_cast<int32_t>(requested_state), static_cast<uint8_t *>(can_frame.data));
    return can_frame;
  }

  int joint_idx_from_node_id(int node_id)
  {
    for (auto &kv : config_mapping_)
    {
      if (node_id == boost::any_cast<CanJointConfig>(kv.second).node_id)
        return boost::any_cast<CanJointConfig>(kv.second).joint_idx;
    }
    return -1;
  }

  void can_rx_callback(CanFrame frame)
  {
    int node_id = (frame.can_id >> 5) & 0x03F;
    int joint_idx = joint_idx_from_node_id(node_id);
    if (joint_idx == -1)
      return;
    int cmd_id = frame.can_id & 0x01F;
    switch (cmd_id)
    {
      case CanSimpleCommands::ODriveHeartbeatMessage:
        joint_states_[joint_idx].axis_error = serializer_.deserialize_uint32(frame.data);
        joint_states_[joint_idx].current_state = serializer_.deserialize_uint32(&frame.data[4]);
        break;
      case CanSimpleCommands::GetEncoderEstimates:
        joint_states_[joint_idx].position = static_cast<double>(serializer_.deserialize_float(frame.data));
        joint_states_[joint_idx].velocity = static_cast<double>(serializer_.deserialize_float(&frame.data[4]));
        if (!joint_states_[joint_idx].initialized)
          joint_states_[joint_idx].initialized = true;
        break;
      case CanSimpleCommands::GetVbusVoltage:
        std::memcpy(srv_data_, frame.data, 8);
        srv_ready_ = true;
        break;
      default:
        break;
    }
  }
  bool handle_set_requested_state(odrive_ros_control::SetRequestedState::Request &req,
                                  odrive_ros_control::SetRequestedState::Response &res)
  {
    ROS_DEBUG_STREAM("handle_set_requested_state: " + req.joint_name);
    if (config_defined(config_mapping_, req.joint_name))
    {
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      std::lock_guard<std::mutex> guard(can_mutex_);
      ros::Duration duration(0.01);
      duration.sleep();
      CanFrame can_frame = make_set_requested_state_command(config->node_id, req.axis_state);
      can_device_->write_async(can_frame);
      duration.sleep();
    }
    return true;
  }

  bool handle_get_current_state(odrive_ros_control::GetCurrentState::Request &req,
                                odrive_ros_control::GetCurrentState::Response &res)
  {
    ROS_DEBUG_STREAM("handle_get_current_state");
    if (config_defined(config_mapping_, req.joint_name))
    {
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      res.axis_state = joint_states_[config->joint_idx].current_state;
    }
    return true;
  }

  bool handle_get_axis_error(odrive_ros_control::GetAxisError::Request &req,
                             odrive_ros_control::GetAxisError::Response &res)
  {
    ROS_DEBUG_STREAM("handle_get_axis_error");
    if (config_defined(config_mapping_, req.joint_name))
    {
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      res.axis_error = joint_states_[config->joint_idx].axis_error;
    }
    return true;
  }

  bool handle_set_pos_setpoint(odrive_ros_control::SetPosSetpoint::Request &req,
                               odrive_ros_control::SetPosSetpoint::Response &res)
  {
    ROS_DEBUG_STREAM("handle_set_pos_setpoint: " + req.joint_name);
    if (config_defined(config_mapping_, req.joint_name))
    {
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      std::lock_guard<std::mutex> guard(can_mutex_);
      ros::Duration duration(0.01);
      duration.sleep();
      CanFrame can_frame = make_position_command(config->node_id, req.pos_setpoint, req.vel_ff);
      can_device_->write_async(can_frame);
      duration.sleep();
    }
    return true;
  }

  bool handle_set_vel_setpoint(odrive_ros_control::SetVelSetpoint::Request &req,
                               odrive_ros_control::SetVelSetpoint::Response &res)
  {
    ROS_DEBUG_STREAM("handle_set_vel_setpoint: " + req.joint_name);
    if (config_defined(config_mapping_, req.joint_name))
    {
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      std::lock_guard<std::mutex> guard(can_mutex_);
      ros::Duration duration(0.01);
      duration.sleep();
      CanFrame can_frame = make_velocity_command(config->node_id, req.vel_setpoint, req.current_ff);
      can_device_->write_async(can_frame);
      duration.sleep();
    }
    return true;
  }

  bool handle_set_current_setpoint(odrive_ros_control::SetCurrentSetpoint::Request &req,
                                   odrive_ros_control::SetCurrentSetpoint::Response &res)
  {
    ROS_DEBUG_STREAM("handle_set_current_setpoint: " + req.joint_name);
    if (config_defined(config_mapping_, req.joint_name))
    {
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      std::lock_guard<std::mutex> guard(can_mutex_);
      ros::Duration duration(0.01);
      duration.sleep();
      CanFrame can_frame = make_current_command(config->node_id, req.current_setpoint);
      can_device_->write_async(can_frame);
      duration.sleep();
    }
    return true;
  }

  bool handle_move_to_pos(odrive_ros_control::MoveToPos::Request &req, odrive_ros_control::MoveToPos::Response &res)
  {
    ROS_DEBUG_STREAM("handle_move_to_pos: " + req.joint_name);
    if (config_defined(config_mapping_, req.joint_name))
    {
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      std::lock_guard<std::mutex> guard(can_mutex_);
      ros::Duration duration(0.01);
      duration.sleep();
      CanFrame can_frame = make_move_to_pos_command(config->node_id, req.position);
      can_device_->write_async(can_frame);
      duration.sleep();
    }
    return true;
  }

  bool handle_get_vbus_voltage(odrive_ros_control::GetVbusVoltage::Request &req,
                               odrive_ros_control::GetVbusVoltage::Response &res)
  {
    if (config_defined(config_mapping_, req.joint_name))
    {
      std::lock_guard<std::mutex> guard(can_mutex_);
      ros::Duration duration(0.01);
      duration.sleep();  // this gives some time for ODrive to process CAN mailbox
      srv_ready_ = false;
      int i = 0;
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      CanFrame can_frame = make_vbus_command(config->node_id);
      can_device_->write_async(can_frame);
      while (true)
      {
        if (srv_ready_ == true)
        {
          res.vbus_voltage = serializer_.deserialize_float(srv_data_);
          ROS_DEBUG_STREAM("vbus:" + std::to_string(res.vbus_voltage));
          res.result = "success";
          break;
        }
        else if (i > 5)
        {
          break;
        }
        ROS_DEBUG_STREAM("handle_get_vbus_voltage loop:" + std::to_string(i));
        i++;
        duration.sleep();
      }
    }
    return true;
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::CanTransport, odrive_ros_control::transport::CommandTransport)