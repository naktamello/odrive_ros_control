/*
 * Author: naktamello
 */
#include <odrive_ros_control/odrive_hardware_interface.h>
#include <chrono>

int main(int argc, char** argv)
{
ROS_INFO_STREAM_NAMED("hardware_interface", "Starting odrive_hardware_interface node");

ros::init(argc, argv, "odrive_hardware_interface");

ros::AsyncSpinner spinner(2);
spinner.start();

ros::NodeHandle nh;

odrive_hardware_interface::ODriveHardwareInterface odrive_hardware_interface;
odrive_hardware_interface.configure();

ros::Time timestamp;
ros::Duration period;
auto stopwatch_last = std::chrono::steady_clock::now();
auto stopwatch_now = stopwatch_last;

controller_manager::ControllerManager controller_manager(&odrive_hardware_interface, nh);
odrive_hardware_interface.start();

timestamp = ros::Time::now();
stopwatch_now = std::chrono::steady_clock::now();
period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now-stopwatch_last).count());
stopwatch_last = stopwatch_now;

while (ros::ok()){
    if (!odrive_hardware_interface.read(timestamp, period))
    {
        ROS_FATAL_NAMED("odrive_hardware_interface", "Failed to read from RobotHW");
        ros::shutdown();
    }

    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now-stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    controller_manager.update(timestamp, period);

    odrive_hardware_interface.write(timestamp, period);


}
    spinner.stop();
    ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down");

    return 0;

}