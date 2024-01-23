// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/TractionDriveCommand.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__TRACTION_DRIVE_COMMAND__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__TRACTION_DRIVE_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__TractionDriveCommand __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__TractionDriveCommand __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TractionDriveCommand_
{
  using Type = TractionDriveCommand_<ContainerAllocator>;

  explicit TractionDriveCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0;
      this->steering = 0.0;
    }
  }

  explicit TractionDriveCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0;
      this->steering = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _speed_type =
    int16_t;
  _speed_type speed;
  using _steering_type =
    double;
  _steering_type steering;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__speed(
    const int16_t & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__steering(
    const double & _arg)
  {
    this->steering = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__TractionDriveCommand
    std::shared_ptr<hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__TractionDriveCommand
    std::shared_ptr<hqv_public_interface::msg::TractionDriveCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TractionDriveCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->steering != other.steering) {
      return false;
    }
    return true;
  }
  bool operator!=(const TractionDriveCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TractionDriveCommand_

// alias to use template instance with default allocator
using TractionDriveCommand =
  hqv_public_interface::msg::TractionDriveCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__TRACTION_DRIVE_COMMAND__STRUCT_HPP_
