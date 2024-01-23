// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerMowerMode.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerMowerMode __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerMowerMode __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerMowerMode_
{
  using Type = MowerMowerMode_<ContainerAllocator>;

  explicit MowerMowerMode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->description = "";
    }
  }

  explicit MowerMowerMode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->description = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mode_type =
    uint8_t;
  _mode_type mode;
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
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MOWERAPP_MODE_AUTO =
    0u;
  static constexpr uint8_t MOWERAPP_MODE_MANUAL =
    1u;
  static constexpr uint8_t MOWERAPP_MODE_HOME =
    2u;
  static constexpr uint8_t MOWERAPP_MODE_DEMO =
    3u;
  static constexpr uint8_t MOWERAPP_MODE_POI =
    4u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerMowerMode
    std::shared_ptr<hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerMowerMode
    std::shared_ptr<hqv_public_interface::msg::MowerMowerMode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerMowerMode_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerMowerMode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerMowerMode_

// alias to use template instance with default allocator
using MowerMowerMode =
  hqv_public_interface::msg::MowerMowerMode_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerMode_<ContainerAllocator>::MOWERAPP_MODE_AUTO;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerMode_<ContainerAllocator>::MOWERAPP_MODE_MANUAL;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerMode_<ContainerAllocator>::MOWERAPP_MODE_HOME;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerMode_<ContainerAllocator>::MOWERAPP_MODE_DEMO;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerMode_<ContainerAllocator>::MOWERAPP_MODE_POI;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__STRUCT_HPP_
