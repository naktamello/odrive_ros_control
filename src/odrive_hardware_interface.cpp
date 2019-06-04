/*
 * Author: naktamello
 */
#include <odrive_ros_control/odrive_hardware_interface.h>
#include <boost/format.hpp>

namespace odrive_hardware_interface{

ODriveHardwareInterface::ODriveHardwareInterface()
//   joint_position_(6, 0.0), joint_velocity_(6, 0.0), joint_effort_(6, 0.0),
//   joint_position_command_(6, 0.0), joint_velocity_command_(6, 0.0), joint_effort_command_(6, 0.0),
//   n_dof_(6)
{
  if (!nh_.getParam("/odrive/hardware_interface/joints", joint_names_)){
      std::string err_msg("You must load joint names to '/odrive/hardware_interface/joints' on param server.");
      ROS_ERROR(err_msg.c_str());
      throw std::runtime_error(err_msg.c_str());
  }
  n_dof_ = joint_names_.size();
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);

  for (std::size_t i=0; i<n_dof_;++i){
      joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i],
                                                                                 &joint_effort_[i]));
      posvel_joint_interface_.registerHandle(hardware_interface::PosVelJointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                                                                                                    &joint_position_command_[i],
                                                                                                                    &joint_velocity_command_[i]));
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&posvel_joint_interface_);

  ROS_INFO_STREAM_NAMED("hardware_interface", "ODriveHardwareInterface loaded");

}
ODriveHardwareInterface::~ODriveHardwareInterface()
{
}

bool ODriveHardwareInterface::read(const ros::Time time, const ros::Duration period){
return true;
}

bool ODriveHardwareInterface::write(const ros::Time time, const ros::Duration period){
  ROS_DEBUG_STREAM(boost::format("ODriveHardwareInterface::write=%1% /  %2%")%joint_position_command_[0]%joint_velocity_command_[0]);
return true;
}

void ODriveHardwareInterface::configure(){

}

void ODriveHardwareInterface::start(){

}

}