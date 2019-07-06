#include <async_comm/async_serial.h>

namespace async_comm
{
SerialDevice::SerialDevice(unsigned int baud, const std::string& device, int out_buffer_len) : ok_(true)
{
  out_buffer_.reserve(out_buffer_len);
  io_service_ = std::make_shared<boost::asio::io_service>();
  serial_port_ = std::make_unique<boost::asio::serial_port>(*io_service_, device);
  if (!serial_port_->is_open())
  {
  }
  boost::asio::serial_port_base::baud_rate baud_option(baud);
  serial_port_->set_option(baud_option);
  boost::thread t(boost::bind(&SerialDevice::start_thread, this));
}

SerialDevice::~SerialDevice()
{
  io_service_->stop();
  serial_port_->close();
}

void SerialDevice::set_recv_callback(RxCallback cb)
{
  recv_callback_ = cb;
}

void SerialDevice::load_buffer(std::string&& msg)
{
  out_buffer_.emplace_back(msg);
}

void SerialDevice::write_buffer()
{
  write_async(std::accumulate(out_buffer_.begin(), out_buffer_.end(), std::string()));
  out_buffer_.clear();
}

void SerialDevice::write_async(std::string&& msg)
{
  boost::asio::async_write(*serial_port_, boost::asio::buffer(msg),
                           boost::bind(&SerialDevice::write_async_complete, this, boost::asio::placeholders::error));
}

void SerialDevice::request_async(std::string&& msg, RxCallback cb)
{
  cb_service_.cb = cb;
  write_async(std::forward<std::string>(msg));
  cb_service_.state = RequestState::WAITING;
}

bool SerialDevice::ok()
{
  return ok_;
}

bool SerialDevice::can_take_request()
{
  return cb_service_.state == RequestState::IDLE;
}

int SerialDevice::loop_count()
{
  return cb_service_.wait_count++;
}

void SerialDevice::reset_request_state()
{
  cb_service_.state = RequestState::IDLE;
  cb_service_.wait_count = 0;
  cb_service_.cb = nullptr;
}

void SerialDevice::start_thread()
{
  start_reading();
  io_service_->run();
}

void SerialDevice::start_reading()
{
  serial_port_->async_read_some(boost::asio::buffer(io_buffer_, max_read_length),
                                boost::bind(&SerialDevice::read_async_complete, this, boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
}

void SerialDevice::read_async_complete(const boost::system::error_code& error, size_t bytes_transferred)
{
  if (!error)
  {
    for (auto i = 0; i < bytes_transferred; ++i)
    {
      if (io_buffer_[i] != '\n')
      {
        if (io_buffer_[i] != '\r')
          read_buffer_.push_back(io_buffer_[i]);
      }
      else
      {
        std::string payload = std::string(std::accumulate(read_buffer_.begin(), read_buffer_.end(), std::string()));
        read_buffer_.clear();
        if (cb_service_.cb)
        {
          cb_service_.cb(payload);
          reset_request_state();
        }
        else if (recv_callback_){
          recv_callback_(payload);
        }
      }
    }
    start_reading();
  }
  else
  {
    terminate(error);
  }
}

void SerialDevice::write_async_complete(const boost::system::error_code& error)
{
  if (!error)
  {
  }
  else
  {
    terminate(error);
  }
}

void SerialDevice::terminate(const boost::system::error_code& error)
{
  if (error == boost::asio::error::operation_aborted)
    return;
  serial_port_->close();
  ok_ = false;
}
}
