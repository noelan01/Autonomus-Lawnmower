// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerBladeMotorV2SpeedMeassure.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_SPEED_MEASSURE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_SPEED_MEASSURE__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerBladeMotorV2SpeedMeassure __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerBladeMotorV2SpeedMeassure __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerBladeMotorV2SpeedMeassure_
{
  using Type = MowerBladeMotorV2SpeedMeassure_<ContainerAllocator>;

  explicit MowerBladeMotorV2SpeedMeassure_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->speed = 0;
    }
  }

  explicit MowerBladeMotorV2SpeedMeassure_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->speed = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _index_type =
    uint8_t;
  _index_type index;
  using _speed_type =
    uint16_t;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__index(
    const uint8_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__speed(
    const uint16_t & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerBladeMotorV2SpeedMeassure
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerBladeMotorV2SpeedMeassure
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerBladeMotorV2SpeedMeassure_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerBladeMotorV2SpeedMeassure_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerBladeMotorV2SpeedMeassure_

// alias to use template instance with default allocator
using MowerBladeMotorV2SpeedMeassure =
  hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_SPEED_MEASSURE__STRUCT_HPP_
