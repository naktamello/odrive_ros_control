#ifndef CAN_SIMPLE_COMMANDS_
#define CAN_SIMPLE_COMMANDS_
namespace odrive_ros_control
{
namespace transport
{
namespace CanSimpleCommands
{
const int ODriveHeartbeatMessage = 0x001;
const int SetAxisRequestedState = 0x007;
const int GetEncoderEstimates = 0x009;
const int MoveToPos = 0x00B;
const int SetPosSetpoint = 0x00C;
const int SetVelSetpoint = 0x00D;
const int SetCurrentSetpoint = 0x00E;
const int GetVbusVoltage = 0x017;
}  // namespace CanSimpleCommands
}  // namespace transport
}  // namespace odrive_ros_control
#endif