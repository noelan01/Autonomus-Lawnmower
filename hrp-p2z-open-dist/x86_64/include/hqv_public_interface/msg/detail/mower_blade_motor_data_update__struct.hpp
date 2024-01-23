// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerBladeMotorDataUpdate.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerBladeMotorDataUpdate __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerBladeMotorDataUpdate __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerBladeMotorDataUpdate_
{
  using Type = MowerBladeMotorDataUpdate_<ContainerAllocator>;

  explicit MowerBladeMotorDataUpdate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->direction = 0;
      this->rpm = 0;
      this->current = 0;
    }
  }

  explicit MowerBladeMotorDataUpdate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->direction = 0;
      this->rpm = 0;
      this->current = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;
  using _direction_type =
    uint8_t;
  _direction_type direction;
  using _rpm_type =
    uint16_t;
  _rpm_type rpm;
  using _current_type =
    uint16_t;
  _current_type current;

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
  Type & set__direction(
    const uint8_t & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__rpm(
    const uint16_t & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__current(
    const uint16_t & _arg)
  {
    this->current = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerBladeMotorDataUpdate
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerBladeMotorDataUpdate
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorDataUpdate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerBladeMotorDataUpdate_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerBladeMotorDataUpdate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerBladeMotorDataUpdate_

// alias to use template instance with default allocator
using MowerBladeMotorDataUpdate =
  hqv_public_interface::msg::MowerBladeMotorDataUpdate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__STRUCT_HPP_
