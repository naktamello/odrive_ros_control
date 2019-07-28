/*
 * Author: naktamello
 */
#include <odrive_ros_control/GetAxisError.h>
#include <odrive_ros_control/GetCurrentState.h>
#include <odrive_ros_control/SetRequestedState.h>
#include <odrive_ros_control/odrive_hardware_interface.h>
#include <chrono>

static const double default_loop_rate = 0.01;  // 10ms

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("hardware_interface", "Starting odrive_hardware_interface node");
  ros::init(argc, argv, "odrive_hardware_interface");
  ros::NodeHandle nh;

  double loop_rate_ms;
  double loop_rate;

  if (nh.getParam(odrive_ros_control::transport::param_prefix + "loop_rate_ms", loop_rate_ms))
  {
    loop_rate = loop_rate_ms / 1000.0;
    ROS_DEBUG_STREAM("loop_rate:" + std::to_string(loop_rate));
  }
  else
  {
    ROS_DEBUG_STREAM("loop_rate: using default value of " + std::to_string(default_loop_rate) + "ms");
    loop_rate = default_loop_rate;
  }

  ros::AsyncSpinner spinner(2);
  spinner.start();

  odrive_hardware_interface::ODriveHardwareInterface odrive_hardware_interface;
  ros::NodeHandle hw_nh("odrive_hardware");
  odrive_hardware_interface.init(nh, hw_nh);
  controller_manager::ControllerManager controller_manager(&odrive_hardware_interface, nh);
  odrive_hardware_interface.start();

  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;
  ros::Duration duration(loop_rate);
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  while (ros::ok())
  {
    odrive_hardware_interface.read(timestamp, period);

    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    controller_manager.update(timestamp, period);

    odrive_hardware_interface.write(timestamp, period);

    duration.sleep();
  }
  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down");

  return 0;
}