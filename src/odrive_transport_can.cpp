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
#include <boost/any.hpp>
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
namespace CanSimpleCommands
{
const int ODriveHeartbeatMessage = 0x001;
const int GetEncoderEstimates = 0x009;
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
      // std::cout << "received:" + std::to_string(bytes_transferred) << std::endl;
      std::memcpy(&frame_, read_buffer_, bytes_transferred);
      // write_async(frame_);
      callback_(frame_);
      print_can_msg(frame_);
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
  }

  bool send(std::vector<double> &position_cmd, std::vector<double> &velocity_cmd)
  {
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (config_defined(config_mapping_, joint_names_[i]))
      {
        CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[joint_names_[i]]);
        CanFrame can_frame = make_position_command(config->node_id, position_cmd[i], velocity_cmd[i]);
        can_device_->write_async(can_frame);
      }
    }
  }
  bool receive(std::vector<double> &position, std::vector<double> &velocity)
  {
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
        can_device_->write_async(can_frame);
      }
    }
  }

private:
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
  CanFrame make_position_command(int node_id, double &position, double &velocity)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | 0x00C;
    serializer_.serialize_int32(static_cast<int32_t>(position), static_cast<uint8_t *>(can_frame.data));
    serializer_.serialize_int32(static_cast<int16_t>(velocity), static_cast<uint8_t *>(&can_frame.data[4]));
    return can_frame;
  }
  CanFrame make_feedback_command(int node_id)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | 0x009 | (1 << 30);
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
    int joint_idx = -1;
    int node_id = (frame.can_id >> 5) & 0x03F;
    int cmd_id = frame.can_id & 0x01F;
    switch (cmd_id)
    {
      case CanSimpleCommands::ODriveHeartbeatMessage:
        break;
      case CanSimpleCommands::GetEncoderEstimates:
        joint_idx = joint_idx_from_node_id(node_id);
        if (joint_idx == -1)
          break;
        joint_states_[joint_idx].position = static_cast<double>(serializer_.deserialize_float(frame.data));
        joint_states_[joint_idx].velocity = static_cast<double>(serializer_.deserialize_float(&frame.data[4]));
        if (!joint_states_[joint_idx].initialized)
          joint_states_[joint_idx].initialized = true;
        break;
      default:
        break;
    }
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::CanTransport, odrive_ros_control::transport::CommandTransport)