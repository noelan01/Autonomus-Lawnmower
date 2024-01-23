// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverResult.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerCoDriverResult __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerCoDriverResult __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerCoDriverResult_
{
  using Type = MowerCoDriverResult_<ContainerAllocator>;

  explicit MowerCoDriverResult_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->command = 0;
      this->resultcode = 0;
    }
  }

  explicit MowerCoDriverResult_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
      this->command = 0;
      this->resultcode = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _result_type =
    uint8_t;
  _result_type result;
  using _command_type =
    uint8_t;
  _command_type command;
  using _resultcode_type =
    uint8_t;
  _resultcode_type resultcode;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__result(
    const uint8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__resultcode(
    const uint8_t & _arg)
  {
    this->resultcode = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ICODRIVER_RESULT_DISABLED =
    0u;
  static constexpr uint8_t ICODRIVER_RESULT_COMMAND_COMPLETED =
    1u;
  static constexpr uint8_t ICODRIVER_RESULT_COMMAND_INCOMPLETE =
    2u;
  static constexpr uint8_t ICODRIVER_RESULT_COMMAND_PENDING =
    3u;
  static constexpr uint8_t ICODRIVER_RESULT_COMMAND_TIMEOUT =
    4u;
  static constexpr uint8_t ICODRIVER_RESULT_COMMAND_ACCEPTED =
    5u;
  static constexpr uint8_t ICODRIVER_RESULT_COMMAND_REJECTED =
    6u;
  static constexpr uint8_t ICODRIVER_RESULT_NO_ACTIVE_COMMAND =
    7u;
  static constexpr uint8_t ICODRIVER_COMMAND_NONE =
    0u;
  static constexpr uint8_t ICODRIVER_COMMAND_ABORT =
    1u;
  static constexpr uint8_t ICODRIVER_COMMAND_ROTATE =
    2u;
  static constexpr uint8_t ICODRIVER_COMMAND_TURN =
    3u;
  static constexpr uint8_t ICODRIVER_COMMAND_MOVE_TO =
    4u;
  static constexpr uint8_t ICODRIVER_COMMAND_REDUCE_SPEED =
    5u;
  static constexpr uint8_t ICODRIVER_COMMAND_FAILED =
    0u;
  static constexpr uint8_t ICODRIVER_COMMAND_PASSED =
    1u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerCoDriverResult
    std::shared_ptr<hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerCoDriverResult
    std::shared_ptr<hqv_public_interface::msg::MowerCoDriverResult_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerCoDriverResult_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->resultcode != other.resultcode) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerCoDriverResult_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerCoDriverResult_

// alias to use template instance with default allocator
using MowerCoDriverResult =
  hqv_public_interface::msg::MowerCoDriverResult_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_RESULT_DISABLED;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_RESULT_COMMAND_COMPLETED;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_RESULT_COMMAND_INCOMPLETE;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_RESULT_COMMAND_PENDING;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_RESULT_COMMAND_TIMEOUT;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_RESULT_COMMAND_ACCEPTED;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_RESULT_COMMAND_REJECTED;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_RESULT_NO_ACTIVE_COMMAND;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_COMMAND_NONE;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_COMMAND_ABORT;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_COMMAND_ROTATE;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_COMMAND_TURN;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_COMMAND_MOVE_TO;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_COMMAND_REDUCE_SPEED;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_COMMAND_FAILED;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverResult_<ContainerAllocator>::ICODRIVER_COMMAND_PASSED;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__STRUCT_HPP_
