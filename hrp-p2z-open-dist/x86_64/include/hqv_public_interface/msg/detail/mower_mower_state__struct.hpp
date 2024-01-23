// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerMowerState.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerMowerState __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerMowerState __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerMowerState_
{
  using Type = MowerMowerState_<ContainerAllocator>;

  explicit MowerMowerState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->description = "";
    }
  }

  explicit MowerMowerState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->description = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MOWERAPP_STATE_OFF =
    0u;
  static constexpr uint8_t MOWERAPP_STATE_WAIT_FOR_SAFETYPIN =
    1u;
  static constexpr uint8_t MOWERAPP_STATE_STOPPED =
    2u;
  static constexpr uint8_t MOWERAPP_STATE_FATAL_ERROR =
    3u;
  static constexpr uint8_t MOWERAPP_STATE_PENDING_START =
    4u;
  static constexpr uint8_t MOWERAPP_STATE_PAUSED =
    5u;
  static constexpr uint8_t MOWERAPP_STATE_IN_OPERATION =
    6u;
  static constexpr uint8_t MOWERAPP_STATE_RESTRICTED =
    7u;
  static constexpr uint8_t MOWERAPP_STATE_ERROR =
    8u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerMowerState_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerMowerState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerMowerState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerMowerState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerMowerState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerMowerState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerMowerState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerMowerState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerMowerState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerMowerState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerMowerState
    std::shared_ptr<hqv_public_interface::msg::MowerMowerState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerMowerState
    std::shared_ptr<hqv_public_interface::msg::MowerMowerState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerMowerState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerMowerState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerMowerState_

// alias to use template instance with default allocator
using MowerMowerState =
  hqv_public_interface::msg::MowerMowerState_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_OFF;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_WAIT_FOR_SAFETYPIN;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_STOPPED;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_FATAL_ERROR;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_PENDING_START;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_PAUSED;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_IN_OPERATION;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_RESTRICTED;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerState_<ContainerAllocator>::MOWERAPP_STATE_ERROR;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__STRUCT_HPP_
