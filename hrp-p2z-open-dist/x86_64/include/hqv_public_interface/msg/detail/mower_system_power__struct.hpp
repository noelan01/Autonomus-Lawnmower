// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerSystemPower.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerSystemPower __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerSystemPower __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerSystemPower_
{
  using Type = MowerSystemPower_<ContainerAllocator>;

  explicit MowerSystemPower_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->countdown = 0ul;
    }
  }

  explicit MowerSystemPower_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->countdown = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _countdown_type =
    uint32_t;
  _countdown_type countdown;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__countdown(
    const uint32_t & _arg)
  {
    this->countdown = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ISYSTEMPOWER_MODE_POWER_UP_AND_INIT =
    0u;
  static constexpr uint8_t ISYSTEMPOWER_MODE_ACTIVE =
    1u;
  static constexpr uint8_t ISYSTEMPOWER_MODE_STANDBY =
    2u;
  static constexpr uint8_t ISYSTEMPOWER_MODE_SHUTDOWN =
    3u;
  static constexpr uint8_t ISYSTEMPOWER_MODE_SWITCH_TO_STANDBY =
    4u;
  static constexpr uint8_t ISYSTEMPOWER_MODE_SWITCH_TO_SHUTDOWN =
    5u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerSystemPower
    std::shared_ptr<hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerSystemPower
    std::shared_ptr<hqv_public_interface::msg::MowerSystemPower_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerSystemPower_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->countdown != other.countdown) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerSystemPower_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerSystemPower_

// alias to use template instance with default allocator
using MowerSystemPower =
  hqv_public_interface::msg::MowerSystemPower_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerSystemPower_<ContainerAllocator>::ISYSTEMPOWER_MODE_POWER_UP_AND_INIT;
template<typename ContainerAllocator>
constexpr uint8_t MowerSystemPower_<ContainerAllocator>::ISYSTEMPOWER_MODE_ACTIVE;
template<typename ContainerAllocator>
constexpr uint8_t MowerSystemPower_<ContainerAllocator>::ISYSTEMPOWER_MODE_STANDBY;
template<typename ContainerAllocator>
constexpr uint8_t MowerSystemPower_<ContainerAllocator>::ISYSTEMPOWER_MODE_SHUTDOWN;
template<typename ContainerAllocator>
constexpr uint8_t MowerSystemPower_<ContainerAllocator>::ISYSTEMPOWER_MODE_SWITCH_TO_STANDBY;
template<typename ContainerAllocator>
constexpr uint8_t MowerSystemPower_<ContainerAllocator>::ISYSTEMPOWER_MODE_SWITCH_TO_SHUTDOWN;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__STRUCT_HPP_
