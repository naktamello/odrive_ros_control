/*
 * Author: naktamello
 */
// stl
#include <iostream>
#include <mutex>
// boost
#include <boost/any.hpp>
#include <boost/format.hpp>
// odrive_ros_control
#include <odrive_ros_control/GetAxisError.h>
#include <odrive_ros_control/GetCurrentState.h>
#include <odrive_ros_control/GetVbusVoltage.h>
#include <odrive_ros_control/MoveToPos.h>
#include <odrive_ros_control/ODriveRawCAN.h>
#include <odrive_ros_control/SetCurrentSetpoint.h>
#include <odrive_ros_control/SetPosSetpoint.h>
#include <odrive_ros_control/SetRequestedState.h>
#include <odrive_ros_control/SetVelSetpoint.h>
#include <odrive_ros_control/transport_interface.h>
#include <odrive_ros_control/can_simple_commands.h>
#include <odrive_ros_control/can_simple_serializer.h>
// async_comm
#include <async_comm/async_can.h>
using namespace async_comm;

namespace odrive_ros_control
{
namespace transport
{

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
    services_.push_back(
        nh_ptr_->advertiseService("/odrive_ros_control/odrive_raw_can", &CanTransport::handle_odrive_raw_can, this));
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
  struct ODriveCanMsg
  {
    CanFrame frame;
    int node_id;
    int cmd_id;
  };
  struct ODriveCanMsg last_msg_;
  // uint8_t srv_data_[8];
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
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::GetEncoderEstimates | (1 << RTR_BIT);
    return can_frame;
  }
  CanFrame make_vbus_command(int node_id)
  {
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = (node_id << 5) | CanSimpleCommands::GetVbusVoltage | (1 << RTR_BIT);
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
      // case CanSimpleCommands::GetVbusVoltage:
      //   std::memcpy(srv_data_, frame.data, 8);
      //   srv_ready_ = true;
      //   break;
      default:
        std::memcpy(&last_msg_.frame, &frame, sizeof(CanFrame));
        last_msg_.node_id = node_id;
        last_msg_.cmd_id = cmd_id;
        // std::memcpy(srv_data_, frame.data, 8);
        srv_ready_ = true;
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
      CanJointConfig *config = boost::any_cast<CanJointConfig>(&config_mapping_[req.joint_name]);
      CanFrame can_frame = make_vbus_command(config->node_id);
      if (wait_for_response(can_frame))
      {
        if (last_msg_.cmd_id == CanSimpleCommands::GetVbusVoltage)
        {
          res.vbus_voltage = serializer_.deserialize_float(last_msg_.frame.data);
          ROS_DEBUG_STREAM("vbus:" + std::to_string(res.vbus_voltage));
          res.result = "success";
        }
      }
    }
    return true;
  }
  bool handle_odrive_raw_can(odrive_ros_control::ODriveRawCAN::Request &req,
                             odrive_ros_control::ODriveRawCAN::Response &res)
  {
    ROS_DEBUG_STREAM("handle_odrive_raw_can");
    res.result = "success";
    CanFrame can_frame{};
    can_frame.can_dlc = 8;
    can_frame.can_id = req.id;
    if (req.rtr)
      can_frame.can_id |= (1 << RTR_BIT);
    std::memcpy(can_frame.data, &req.data[0], 8);
    if (req.has_response)
    {
      if (wait_for_response(can_frame))
      {
        std::memcpy(&res.data[0], last_msg_.frame.data, 8);
        ROS_DEBUG_STREAM("response received. node_id:" + std::to_string(last_msg_.node_id) + " cmd_id:" +
                         std::to_string(last_msg_.cmd_id));
        return true;
      }
      ROS_DEBUG_STREAM("no response returning false");
      res.result = "timeout";
    }
    else
      safe_write(can_frame);
    return true;
  }

  void safe_write(CanFrame &frame)
  {
    ROS_DEBUG_STREAM("sending CanFrame");
    std::lock_guard<std::mutex> guard(can_mutex_);
    ros::Duration duration(0.01);  // this gives some time for ODrive to process CAN mailbox
    duration.sleep();
    can_device_->write_async(frame);
    duration.sleep();
  }

  bool wait_for_response(CanFrame &frame)
  {
    std::lock_guard<std::mutex> guard(can_mutex_);
    ros::Duration duration(0.01);  // this gives some time for ODrive to process CAN mailbox
    duration.sleep();
    srv_ready_ = false;
    can_device_->write_async(frame);
    for (size_t i = 0; i < 5; ++i)
    {
      if (srv_ready_ == true)
      {
        return true;
      }
      ROS_DEBUG_STREAM("wait_for_response loop:" + std::to_string(i));
      duration.sleep();
    }
    return false;
  }
};
}
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::CanTransport, odrive_ros_control::transport::CommandTransport)