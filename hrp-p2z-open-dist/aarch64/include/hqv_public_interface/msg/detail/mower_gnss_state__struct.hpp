// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerGnssState.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerGnssState __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerGnssState __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerGnssState_
{
  using Type = MowerGnssState_<ContainerAllocator>;

  explicit MowerGnssState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit MowerGnssState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;

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

  // constant declarations
  static constexpr uint8_t IGNSS_STATE_STARTUP =
    0u;
  static constexpr uint8_t IGNSS_STATE_OK =
    1u;
  static constexpr uint8_t IGNSS_STATE_HALTED =
    2u;
  static constexpr uint8_t IGNSS_STATE_ERROR =
    3u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerGnssState_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerGnssState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssState
    std::shared_ptr<hqv_public_interface::msg::MowerGnssState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssState
    std::shared_ptr<hqv_public_interface::msg::MowerGnssState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerGnssState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerGnssState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerGnssState_

// alias to use template instance with default allocator
using MowerGnssState =
  hqv_public_interface::msg::MowerGnssState_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssState_<ContainerAllocator>::IGNSS_STATE_STARTUP;
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssState_<ContainerAllocator>::IGNSS_STATE_OK;
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssState_<ContainerAllocator>::IGNSS_STATE_HALTED;
template<typename ContainerAllocator>
constexpr uint8_t MowerGnssState_<ContainerAllocator>::IGNSS_STATE_ERROR;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__STRUCT_HPP_
