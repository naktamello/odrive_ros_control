/*
 * Author: naktamello
 */

#include <odrive_ros_control/odrive_hardware_sim.h>
#include <urdf/model.h>

namespace odrive_ros_control
{

bool ODriveHardwareSim::initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
    {
        ROS_INFO("ODriveHardwareSim::initSim() called");
        ROS_DEBUG_STREAM("ODriveHardwareSim::initSim() called");
        // ROS_DEBUG_NAMED("ODriveHardwareSim", "ODriveHardwareSim::initSim() called");

        n_dof_ = transmissions.size();
        joint_names_.resize(n_dof_);
        joint_position_.resize(n_dof_);
        joint_velocity_.resize(n_dof_);
        joint_effort_.resize(n_dof_);
        joint_position_command_.resize(n_dof_);
        joint_velocity_command_.resize(n_dof_);
        joint_effort_command_.resize(n_dof_);
        
        for (std::size_t i=0; i< n_dof_;++i){
        std::vector<std::string> joint_interfaces = transmissions[i].joints_[0].hardware_interfaces_;
        joint_names_[i] = transmissions[i].joints_[0].name_;
        joint_position_[i] = 1.0;
        joint_velocity_[i] = 0.0;
        joint_effort_[i] = 1.0;
        joint_position_command_[i] = 0.0;
        joint_velocity_command_[i] = 0.0;
        joint_effort_command_[i] = 0.0;
        const std::string& hardware_interface = joint_interfaces.front();
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
            joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
        posvel_joint_interface_.registerHandle(hardware_interface::PosVelJointHandle(joint_state_interface_.getHandle(joint_names_[i]),
                                                                                                                    &joint_position_command_[i],
                                                                                                                    &joint_velocity_command_[i]));
        gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[i]);
        sim_joints_.push_back(joint);

        #if GAZEBO_MAJOR_VERSION >= 8
        gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
        #else
        gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
        #endif
        physics_type_ = physics->GetType();
        }
        registerInterface(&joint_state_interface_);
        registerInterface(&posvel_joint_interface_);
        e_stop_active_=false;
        last_e_stop_active_=false;

        return true;

    }

    void ODriveHardwareSim::readSim(ros::Time time, ros::Duration period){
        for(std::size_t i=0; i<n_dof_;++i){
            #if GAZEBO_MAJOR_VERSION >= 8
            double position = sim_joints_[i]->Position(0);
            #else
            double position = sim_joints_[i]->GetAngle(0).Radian();
            #endif
            joint_position_[i] += angles::shortest_angular_distance(joint_position_[i], position);
            joint_velocity_[i] = sim_joints_[i]->GetVelocity(0);
            joint_effort_[i] = sim_joints_[i]->GetForce(0);
        }
    }

    void ODriveHardwareSim::writeSim(ros::Time time, ros::Duration period){
        if (e_stop_active_)
        {
            if(!last_e_stop_active_)
            {
                last_joint_position_command_ = joint_position_;
                last_e_stop_active_ = true;
            }
            joint_position_command_ = last_joint_position_command_;
        }
        else{
            last_e_stop_active_ = false;
        }

    for (std::size_t i=0; i < n_dof_; ++i){
        sim_joints_[i]->SetPosition(0, joint_position_command_[i]);
        if (physics_type_.compare("ode") == 0){
            sim_joints_[i]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[i]);
        }
        else{
            sim_joints_[i]->SetVelocity(0, e_stop_active_ ? 0: joint_velocity_command_[i]);
        }
    }

    }

    void ODriveHardwareSim::eStopActive(const bool active){
        e_stop_active_ = active;
    }

}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::ODriveHardwareSim, gazebo_ros_control::RobotHWSim)