/*
 * Author: naktamello
 */
#include <vector>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace odrive_ros_control{
    namespace transport{
        enum TransportType
        {
            UART = 0,
            CAN = 1,
            USB_NATIVE = 2
        };

        class CommandTransport{
            public:
            //   CommandTransport(std::string param_namespace):
              virtual bool init_transport(std::string param_namespace){
                  param_namepsace_ = param_namespace;
              };
              virtual ~CommandTransport() {}
              virtual bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd) = 0;
              virtual bool receive(std::vector<double>& position) = 0;

            private:
              std::string param_namepsace_;
        };
    }
}