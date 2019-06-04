/*
 * Author: naktamello
 */
#include <odrive_ros_control/transport_interface.h>

namespace odrive_ros_control{
    namespace transport{
        class UartTransport: public CommandTransport{
            using CommandTransport::init_transport;
            public:
              bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd){
                  ROS_DEBUG_STREAM(boost::format("UartTransport::send()=%1% /  %2%")%position_cmd[0]%velocity_cmd[0]);
                //  ROS_DEBUG_STREAM("UartTransport::send()");
              }
              bool receive(std::vector<double>& position) {


              }


        };
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::UartTransport, odrive_ros_control::transport::CommandTransport)