/*
 * Author: naktamello
 */

#ifndef ODRIVE_ROS_CONTROL_ODRIVE_HARDWARE_INTERFACE_
#define ODRIVE_ROS_CONTROL_ODRIVE_HARDWARE_INTERFACE_
// std lib
#include <vector>
#include <string>
// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace odrive_hardware_interface
{

class ODriveHardwareInterface: public hardware_interface::RobotHW

{
public:
  ODriveHardwareInterface();
  ~ODriveHardwareInterface();

  void start();
  void configure();
  bool read(const ros::Time time, const ros::Duration period);
  bool write(const ros::Time time, const ros::Duration period);
protected:
  ros::NodeHandle nh_;
  unsigned int n_dof_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

  hardware_interface::JointStateInterface joint_state_interface_;
//   hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::PosVelJointInterface posvel_joint_interface_;
//   hardware_interface::
};

}

#endif