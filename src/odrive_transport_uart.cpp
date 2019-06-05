/*
 * Author: naktamello
 */
#include <odrive_ros_control/transport_interface.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <deque>
#include <iostream>

namespace odrive_ros_control
{
namespace transport
{
/*
 * SerialDevice class is adaptation of async serial port example by Jeff Gray(2008).
 * http://boost.2283326.n4.nabble.com/Simple-serial-port-demonstration-with-boost-asio-asynchronous-I-O-td2582657.html
 */
class SerialDevice
{
public:
  SerialDevice(boost::asio::io_service& io_service, unsigned int baud, const std::string& device)
    : active_(true), io_service_(io_service), serial_port_(io_service, device)
  {
    if (!serial_port_.is_open())
    {
      throw std::runtime_error("Cannot open serial port!");
    }
    boost::asio::serial_port_base::baud_rate baud_option(baud);
    serial_port_.set_option(baud_option);
    read_start();
  }

  void write(const char msg)  // pass the write data to the do_write function via the io service in the other thread
  {
    io_service_.post(boost::bind(&SerialDevice::do_write, this, msg));
  }

  void write_async(std::string&& msg)
  {
    boost::asio::async_write(serial_port_, boost::asio::buffer(msg),
                             boost::bind(&SerialDevice::write_async_complete, this, boost::asio::placeholders::error));
  }

  void close()  // call the do_close function via the io service in the other thread
  {
    io_service_.post(boost::bind(&SerialDevice::do_close, this, boost::system::error_code()));
  }

  bool active()  // return true if the socket is still active
  {
    return active_;
  }
  boost::asio::serial_port serial_port_;  // the serial port this instance is connected to

private:
  static const int max_read_length = 512;  // maximum amount of data to read in one operation

  void read_start(void)
  {  // Start an asynchronous read and call read_complete when it completes or fails
    serial_port_.async_read_some(boost::asio::buffer(read_msg_, max_read_length),
                                 boost::bind(&SerialDevice::read_complete, this, boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));
  }

  void read_complete(const boost::system::error_code& error, size_t bytes_transferred)
  {  // the asynchronous read operation has now completed or failed and returned an error
    // ROS_DEBUG_STREAM("read complete" << std::strlen(read_msg_));
    if (!error)
    {
      for (auto i = 0; i < bytes_transferred; ++i)
      {
        if (read_msg_[i] != '\n')
          buffer_.push_back(read_msg_[i]);
        else
        {
          std::string payload = std::string(std::accumulate(buffer_.begin(), buffer_.end(), std::string()));
          ROS_DEBUG_STREAM("read complete:"<<payload);
          buffer_.clear();
        }
      }                                               // read completed, so process the data
      std::cout.write(read_msg_, bytes_transferred);  // echo to standard output
      read_start();                                   // start waiting for another asynchronous read again
    }
    else
      do_close(error);
  }

  void do_write(const char msg)
  {                                                 // callback to handle write call from outside this class
    bool write_in_progress = !write_msgs_.empty();  // is there anything currently being written?
    write_msgs_.push_back(msg);                     // store in write buffer
    if (!write_in_progress)                         // if nothing is currently being written, then start
      write_start();
  }

  void write_start(void)
  {  // Start an asynchronous write and call write_complete when it completes or fails
    boost::asio::async_write(serial_port_, boost::asio::buffer(&write_msgs_.front(), 1),
                             boost::bind(&SerialDevice::write_complete, this, boost::asio::placeholders::error));
  }

  void write_async_complete(const boost::system::error_code& error)
  {
    // ROS_DEBUG_STREAM("write_async_complete");
  }

  void write_complete(const boost::system::error_code& error)
  {  // the asynchronous read operation has now completed or failed and returned an error
    if (!error)
    {  // write completed, so send next write data
      ROS_DEBUG_STREAM("before" << write_msgs_.size());
      write_msgs_.pop_front();  // remove the completed data
      ROS_DEBUG_STREAM("after" << write_msgs_.size());
      if (!write_msgs_.empty())  // if there is anthing left to be written
        write_start();           // then start sending the next item in the buffer
    }
    else
      do_close(error);
  }

  void do_close(const boost::system::error_code& error)
  {  // something has gone wrong, so close the socket & make this object inactive
    if (error == boost::asio::error::operation_aborted)  // if this call is the result of a timer cancel()
      return;                                            // ignore it because the connection cancelled the timer
    if (error)
      std::cerr << "Error: " << error.message() << std::endl;  // show the error message
    else
      std::cout << "Error: Connection did not succeed.\n";
    std::cout << "Press Enter to exit\n";
    serial_port_.close();
    active_ = false;
  }

  bool active_;                          // remains true while this object is still operating
  boost::asio::io_service& io_service_;  // the main IO service that runs this connection
  char read_msg_[max_read_length];       // data read from the socket
  std::deque<char> write_msgs_;          // buffered write data
  boost::circular_buffer<char> buffer_{96};
};

class UartTransport : public CommandTransport
{
  using CommandTransport::init_transport;
  static const int default_baud = 115200;

public:
  bool init_transport(ros::NodeHandle& nh, std::string param_namespace, std::vector<std::string>& joint_names)
  {
    CommandTransport::init_transport(nh, param_namespace, joint_names);
    int baud;
    if (!nh_ptr_->getParam(param_namepsace_ + param_prefix + "uart/baud", baud))
    {
      baud = default_baud;
      ROS_DEBUG_STREAM_NAMED("UartTransport", "using default baudrate");
    }
    std::string port;
    if (!nh_ptr_->getParam(param_namepsace_ + param_prefix + "uart/port", port))
    {
      ROS_FATAL_STREAM_NAMED("UartTransport", "you must specify uart port on param server!"
                                                  << param_namepsace_ + param_prefix + "uart/port");
      ros::shutdown();
    }
    io_service_ = std::make_shared<boost::asio::io_service>();
    serial_device_ = std::unique_ptr<SerialDevice>(new SerialDevice(*io_service_, baud, port));
    boost::thread t(boost::bind(&boost::asio::io_service::run, *&io_service_));
  }

  bool send(std::vector<double>& position_cmd, std::vector<double>& velocity_cmd)
  {
    // TODO map joint to serial port
    serial_device_->write_async(boost::str(pos_cmd_fmt_ % 0 % position_cmd[0] % velocity_cmd[0]));
    // serial_device_->serial_port_.write_some(
    // boost::asio::buffer(boost::str(pos_cmd_fmt_ % 0 % position_cmd[0] % velocity_cmd[0])));
  }
  bool receive(std::vector<double>& position)
  {
    serial_device_->write_async(std::string("f 0\n"));
    // serial_device_->serial_port_.write_some(boost::asio::buffer(std::string("f 0\n")));
  }

private:
  std::unique_ptr<SerialDevice> serial_device_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  boost::basic_format<char> pos_cmd_fmt_ = boost::format("p %1% %2$.4f %3$.4f 0\n");
};
}
}

// int main(int argc, char* argv[])
// {
//   try
//   {
//     if (argc != 3)
//     {
//       std::cerr << "Usage: minicom <baud> <device>\n";
//       return 1;
//     }
//     boost::asio::io_service io_service;
//     // define an instance of the main class of this program
//     odrive_ros_control::transport::SerialDevice c(io_service, boost::lexical_cast<unsigned int>(argv[1]), argv[2]);
//     // run the IO service as a separate thread, so the main thread can block on standard input
//     boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));
//     while (c.active())  // check the internal state of the connection to make sure it's still running
//     {
//       char ch;
//       std::cin.get(ch);  // blocking wait for standard input
//       if (ch == 3)       // ctrl-C to end program
//         break;
//       c.write(ch);
//     }
//     c.close();  // close the minicom client connection
//     t.join();   // wait for the IO service thread to close
//   }
//   catch (std::exception& e)
//   {
//     std::cerr << "Exception: " << e.what() << "\n";
//   }
//   return 0;
// }

PLUGINLIB_EXPORT_CLASS(odrive_ros_control::transport::UartTransport, odrive_ros_control::transport::CommandTransport)