/*
 * Author: naktamello
 */

#ifndef ODRIVE_ROS_CONTROL_ODRIVE_HARDWARE_SIM_
#define ODRIVE_ROS_CONTROL_ODRIVE_HARDWARE_SIM_
// ros control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>
// URDF
#include <urdf/model.h>

namespace odrive_ros_control
{

class ODriveHardwareSim: public gazebo_ros_control::RobotHWSim
{
    public:
    
    virtual bool initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions);
    
    virtual void readSim(ros::Time time, ros::Duration period);

    virtual void writeSim(ros::Time time, ros::Duration period);

    virtual void eStopActive(const bool active);

    protected:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PosVelJointInterface posvel_joint_interface_;

    unsigned int n_dof_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<double> last_joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> last_joint_velocity_command_;
    std::vector<double> joint_effort_command_;

    std::vector<gazebo::physics::JointPtr> sim_joints_;

    std::string physics_type_;

    bool e_stop_active_, last_e_stop_active_;
};

}


#endif