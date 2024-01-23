// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerBladeMotorV2Current.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_CURRENT__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_CURRENT__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerBladeMotorV2Current __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerBladeMotorV2Current __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerBladeMotorV2Current_
{
  using Type = MowerBladeMotorV2Current_<ContainerAllocator>;

  explicit MowerBladeMotorV2Current_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->current = 0;
    }
  }

  explicit MowerBladeMotorV2Current_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->current = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _index_type =
    uint8_t;
  _index_type index;
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
  Type & set__index(
    const uint8_t & _arg)
  {
    this->index = _arg;
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
    hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerBladeMotorV2Current
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerBladeMotorV2Current
    std::shared_ptr<hqv_public_interface::msg::MowerBladeMotorV2Current_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerBladeMotorV2Current_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerBladeMotorV2Current_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerBladeMotorV2Current_

// alias to use template instance with default allocator
using MowerBladeMotorV2Current =
  hqv_public_interface::msg::MowerBladeMotorV2Current_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_CURRENT__STRUCT_HPP_
