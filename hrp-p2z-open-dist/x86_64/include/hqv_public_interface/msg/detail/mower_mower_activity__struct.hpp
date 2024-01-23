// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerMowerActivity.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerMowerActivity __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerMowerActivity __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerMowerActivity_
{
  using Type = MowerMowerActivity_<ContainerAllocator>;

  explicit MowerMowerActivity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->activity = 0;
      this->description = "";
    }
  }

  explicit MowerMowerActivity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->activity = 0;
      this->description = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _activity_type =
    uint8_t;
  _activity_type activity;
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
  Type & set__activity(
    const uint8_t & _arg)
  {
    this->activity = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MOWERAPP_ACTIVITY_NONE =
    0u;
  static constexpr uint8_t MOWERAPP_ACTIVITY_CHARGING =
    1u;
  static constexpr uint8_t MOWERAPP_ACTIVITY_GOING_OUT =
    2u;
  static constexpr uint8_t MOWERAPP_ACTIVITY_MOWING =
    3u;
  static constexpr uint8_t MOWERAPP_ACTIVITY_GOING_HOME =
    4u;
  static constexpr uint8_t MOWERAPP_ACTIVITY_PARKED =
    5u;
  static constexpr uint8_t MOWERAPP_ACTIVITY_STOPPED_IN_GARDEN =
    6u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerMowerActivity
    std::shared_ptr<hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerMowerActivity
    std::shared_ptr<hqv_public_interface::msg::MowerMowerActivity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerMowerActivity_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->activity != other.activity) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerMowerActivity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerMowerActivity_

// alias to use template instance with default allocator
using MowerMowerActivity =
  hqv_public_interface::msg::MowerMowerActivity_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerActivity_<ContainerAllocator>::MOWERAPP_ACTIVITY_NONE;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerActivity_<ContainerAllocator>::MOWERAPP_ACTIVITY_CHARGING;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerActivity_<ContainerAllocator>::MOWERAPP_ACTIVITY_GOING_OUT;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerActivity_<ContainerAllocator>::MOWERAPP_ACTIVITY_MOWING;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerActivity_<ContainerAllocator>::MOWERAPP_ACTIVITY_GOING_HOME;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerActivity_<ContainerAllocator>::MOWERAPP_ACTIVITY_PARKED;
template<typename ContainerAllocator>
constexpr uint8_t MowerMowerActivity_<ContainerAllocator>::MOWERAPP_ACTIVITY_STOPPED_IN_GARDEN;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__STRUCT_HPP_
