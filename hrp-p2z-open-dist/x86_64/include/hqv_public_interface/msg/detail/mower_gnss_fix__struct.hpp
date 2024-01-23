// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerGnssFix.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_FIX__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_FIX__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MowerGnssFix __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerGnssFix __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerGnssFix_
{
  using Type = MowerGnssFix_<ContainerAllocator>;

  explicit MowerGnssFix_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit MowerGnssFix_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t IGNSS_GPSFIX_NO_FIX =
    0u;
  static constexpr uint8_t IGNSS_GPSFIX_DEAD_REACKONING =
    1u;
  static constexpr uint8_t IGNSS_GPSFIX_2D_FIX =
    2u;
  static constexpr uint8_t IGNSS_GPSFIX_3D_FIX =
    3u;
  static constexpr uint8_t IGNSS_GPSFIX_GPS_DEAD_REACKONING =
    4u;
  static constexpr uint8_t IGNSS_GPSFIX_TIME_ONLY =
    5u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssFix
    std::shared_ptr<hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssFix
    std::shared_ptr<hqv_public_interface::msg::MowerGnssFix_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerGnssFix_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerGnssFix_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerGnssFix_

// alias to use template instance with default allocator
using MowerGnssFix =
  hqv_public_interface::msg::MowerGnssFix_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssFix_<ContainerAllocator>::IGNSS_GPSFIX_NO_FIX;
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssFix_<ContainerAllocator>::IGNSS_GPSFIX_DEAD_REACKONING;
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssFix_<ContainerAllocator>::IGNSS_GPSFIX_2D_FIX;
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssFix_<ContainerAllocator>::IGNSS_GPSFIX_3D_FIX;
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssFix_<ContainerAllocator>::IGNSS_GPSFIX_GPS_DEAD_REACKONING;
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssFix_<ContainerAllocator>::IGNSS_GPSFIX_TIME_ONLY;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_FIX__STRUCT_HPP_
