/*
 * Author: naktamello
 */
#include <odrive_ros_control/odrive_hardware_interface.h>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>

namespace odrive_hardware_interface{

ODriveHardwareInterface::ODriveHardwareInterface()
{
  if (!nh_.getParam("/odrive/hardware_interface/joints", joint_names_)){
      ROS_FATAL_STREAM_NAMED("odrive_ros_control","You must load joint names to '/odrive/hardware_interface/joints' on param server.");
      ros::shutdown();
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
  command_transport_->receive(joint_position_);
return true;
}

bool ODriveHardwareInterface::write(const ros::Time time, const ros::Duration period){
  command_transport_->send(joint_position_command_, joint_velocity_command_);
  // ROS_DEBUG_STREAM(boost::format("ODriveHardwareInterface::write=%1% /  %2%")%joint_position_command_[0]%joint_velocity_command_[0]);
return true;
}

void ODriveHardwareInterface::configure(){

}

void ODriveHardwareInterface::start(){
  try{
    transport_loader_.reset
    (new pluginlib::ClassLoader<odrive_ros_control::transport::CommandTransport>
      ("odrive_ros_control", "odrive_ros_control::transport::CommandTransport"));
      command_transport_ = transport_loader_->createInstance("odrive_ros_control/UartTransport");
      command_transport_->init_transport(nh_, "", joint_names_);
      ROS_DEBUG_STREAM("UartTransport loaded");
  }
  catch(pluginlib::LibraryLoadException &ex){
    ROS_FATAL_STREAM_NAMED("odrive_ros_control", "Failed to load odrive transport plugin: " <<ex.what());
    ros::shutdown();
  }
}

}