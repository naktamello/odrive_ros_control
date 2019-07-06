#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>

namespace async_comm
{
using RxCallback = std::function<void(std::string)>;
class SerialDevice
{
public:
  SerialDevice(unsigned int baud, const std::string& device, int out_buffer_len);
  ~SerialDevice();
  void set_recv_callback(RxCallback cb);
  void load_buffer(std::string&& msg);
  void write_buffer();
  void write_async(std::string&& msg);
  void request_async(std::string&& msg, RxCallback cb);
  bool ok();
  bool can_take_request();
  int loop_count();
  void reset_request_state();

private:
  void start_thread();
  void start_reading();
  void read_async_complete(const boost::system::error_code& error, size_t bytes_transferred);
  void write_async_complete(const boost::system::error_code& error);
    void terminate(const boost::system::error_code& error);
  enum class RequestState
  {
    IDLE = 0,
    WAITING = 1
  };
  struct CallbackService
  {
    int wait_count;
    RequestState state;
    RxCallback cb;
  };
  RxCallback recv_callback_ = nullptr;
  // constants
  static const int max_read_length = 512;
  // pointers
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  // state
  bool ok_;
  CallbackService cb_service_ = { 0, RequestState::IDLE, nullptr };
  // buffers
  char io_buffer_[max_read_length];
  boost::circular_buffer<char> read_buffer_{ max_read_length };
  std::vector<std::string> out_buffer_;
};
}
