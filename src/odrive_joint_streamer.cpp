/*
 * Author: naktamello
 */
#include <industrial_robot_client/joint_trajectory_streamer.h>
#include <industrial_utils/param_utils.h>

using industrial_robot_client::joint_trajectory_streamer::JointTrajectoryStreamer;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

class ODriveJointTrajectoryStreamer: public JointTrajectoryStreamer
{
    using JointTrajectoryStreamer::init;
    public:
    bool init(std::string default_ip = "", int default_port=StandardSocketPorts::MOTION)
    {
        ROS_DEBUG_STREAM("Starting ODriveJointTrajectoryStreamer");
        return JointTrajectoryStreamer::init(default_ip, default_port);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_interface");
    ROS_DEBUG_STREAM("in main of odrive_joint_streamer.cpp");
    ODriveJointTrajectoryStreamer motion_interface;
    // motion_interface.init("localhost", 1978);
    ROS_DEBUG_STREAM("odrive_joint_streamer.cpp: before calling init()");
    motion_interface.init();
    ROS_DEBUG_STREAM("odrive_joint_streamer.cpp: after calling init()");
    motion_interface.run();

    return 0;
}