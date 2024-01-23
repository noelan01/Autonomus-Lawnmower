// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerAppCommand.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_APP_COMMAND__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_APP_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerAppCommand __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerAppCommand __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerAppCommand_
{
  using Type = MowerAppCommand_<ContainerAllocator>;

  explicit MowerAppCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  explicit MowerAppCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _command_type =
    uint8_t;
  _command_type command;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t START =
    0u;
  static constexpr uint8_t PAUSE =
    1u;
  static constexpr uint8_t ABORT_START =
    2u;
  static constexpr uint8_t CONFIRM_ERROR =
    3u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerAppCommand
    std::shared_ptr<hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerAppCommand
    std::shared_ptr<hqv_public_interface::msg::MowerAppCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerAppCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerAppCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerAppCommand_

// alias to use template instance with default allocator
using MowerAppCommand =
  hqv_public_interface::msg::MowerAppCommand_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerAppCommand_<ContainerAllocator>::START;
template<typename ContainerAllocator>
constexpr uint8_t MowerAppCommand_<ContainerAllocator>::PAUSE;
template<typename ContainerAllocator>
constexpr uint8_t MowerAppCommand_<ContainerAllocator>::ABORT_START;
template<typename ContainerAllocator>
constexpr uint8_t MowerAppCommand_<ContainerAllocator>::CONFIRM_ERROR;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_APP_COMMAND__STRUCT_HPP_
