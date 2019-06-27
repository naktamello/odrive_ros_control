/*
 * Author: naktamello
 */
#include <odrive_ros_control/GetAxisError.h>
#include <odrive_ros_control/GetCurrentState.h>
#include <odrive_ros_control/SetRequestedState.h>
#include <odrive_ros_control/odrive_hardware_interface.h>
#include <chrono>

static const double default_loop_rate = 0.01;  // 10ms

bool handle_set_requested_state(odrive_ros_control::SetRequestedState::Request& req,
                                odrive_ros_control::SetRequestedState::Response& res)
{
  ROS_DEBUG_STREAM("handle_set_requested_state: " + req.joint_name);
}

bool handle_get_current_state(odrive_ros_control::GetCurrentState::Request& req,
                              odrive_ros_control::GetCurrentState::Response& res)
{
}

bool handle_get_axis_error(odrive_ros_control::GetAxisError::Request& req,
                           odrive_ros_control::GetAxisError::Response& res)
{
}

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

//   ros::ServiceServer service1 =
    //   nh.advertiseService("/odrive_ros_control/set_requested_state", handle_set_requested_state);
//   ros::ServiceServer service2 = nh.advertiseService("/odrive_ros_control/get_current_state", handle_get_current_state);
//   ros::ServiceServer service3 = nh.advertiseService("/odrive_ros_control/get_axis_error", handle_get_axis_error);

  odrive_hardware_interface::ODriveHardwareInterface odrive_hardware_interface;
  odrive_hardware_interface.configure();
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
    if (!odrive_hardware_interface.read(timestamp, period))
    {
      ROS_FATAL_NAMED("odrive_hardware_interface", "Failed to read from RobotHW");
      ros::shutdown();
    }

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