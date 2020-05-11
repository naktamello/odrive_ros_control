#ifndef CAN_SIMPLE_SERIALIZER_
#define CAN_SIMPLE_SERIALIZER_
#include <limits>
#include <stdint.h>

namespace odrive_ros_control
{
namespace transport
{
static const int RTR_BIT = 30;
enum class Endianness
{
  LITTLE = 0,
  BIG = 1
};
class CanSimpleSerializer
{
public:
  CanSimpleSerializer()
  {
    static_assert(std::numeric_limits<float>::is_iec559, "float type is not IEEE754");
    endian_ = byte_order();
  }

  template <typename T>
  void serialize(const T &value, uint8_t *dst)
  {
    auto src = reinterpret_cast<const uint8_t *>(&value);
    endian_copy(src, dst, sizeof(T));
  }

  template <typename T>
  T deserialize(uint8_t *src)
  {
    T dst;
    endian_copy(src, reinterpret_cast<uint8_t *>(&dst), sizeof(T));
    return dst;
  };

  void serialize_float(const float &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  float deserialize_float(uint8_t *src)
  {
    return deserialize<float>(src);
  }

  void serialize_uint32(const uint32_t &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  uint32_t deserialize_uint32(uint8_t *src)
  {
    return deserialize<uint32_t>(src);
  }

  void serialize_int32(const int32_t &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  int32_t deserialize_int32(uint8_t *src)
  {
    return deserialize<int32_t>(src);
  }

  void serialize_uint16(const uint16_t &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  uint16_t deserialize_uint16(uint8_t *src)
  {
    return deserialize<uint16_t>(src);
  }

  void serialize_int16(const int16_t &value, uint8_t *dst)
  {
    serialize(value, dst);
  }

  int16_t deserialize_int16(uint8_t *src)
  {
    return deserialize<int16_t>(src);
  }

  void endian_copy(const uint8_t *src, uint8_t *dst, size_t size)
  {
    if (endian_ == Endianness::LITTLE)
      std::memcpy(dst, src, size);
    else
    {
      size_t offset = size - 1;
      for (std::size_t i = 0; i < size; ++i)
      {
        *dst++ = *(src + offset--);
      }
    }
  }

  Endianness byte_order()
  {
    short int word = 0x00001;
    char *b = (char *)&word;
    return (b[0] ? Endianness::LITTLE : Endianness::BIG);
  }

private:
  Endianness endian_;
};
}
}

#endif