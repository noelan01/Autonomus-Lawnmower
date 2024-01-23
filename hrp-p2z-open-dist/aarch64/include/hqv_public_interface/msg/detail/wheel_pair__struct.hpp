// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/WheelPair.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__WheelPair __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__WheelPair __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WheelPair_
{
  using Type = WheelPair_<ContainerAllocator>;

  explicit WheelPair_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->power_left = 0;
      this->power_right = 0;
      this->counter_left = 0l;
      this->counter_right = 0l;
      this->free_left = 0;
      this->free_right = 0;
      this->deltadistance_left = 0.0f;
      this->deltadistance_right = 0.0f;
      this->state_left = 0;
      this->state_right = 0;
      this->current_left = 0;
      this->current_right = 0;
      this->speed_left = 0.0f;
      this->speed_right = 0.0f;
      this->isrunning_left = false;
      this->isrunning_right = false;
    }
  }

  explicit WheelPair_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->power_left = 0;
      this->power_right = 0;
      this->counter_left = 0l;
      this->counter_right = 0l;
      this->free_left = 0;
      this->free_right = 0;
      this->deltadistance_left = 0.0f;
      this->deltadistance_right = 0.0f;
      this->state_left = 0;
      this->state_right = 0;
      this->current_left = 0;
      this->current_right = 0;
      this->speed_left = 0.0f;
      this->speed_right = 0.0f;
      this->isrunning_left = false;
      this->isrunning_right = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _power_left_type =
    uint16_t;
  _power_left_type power_left;
  using _power_right_type =
    uint16_t;
  _power_right_type power_right;
  using _counter_left_type =
    int32_t;
  _counter_left_type counter_left;
  using _counter_right_type =
    int32_t;
  _counter_right_type counter_right;
  using _free_left_type =
    uint8_t;
  _free_left_type free_left;
  using _free_right_type =
    uint8_t;
  _free_right_type free_right;
  using _deltadistance_left_type =
    float;
  _deltadistance_left_type deltadistance_left;
  using _deltadistance_right_type =
    float;
  _deltadistance_right_type deltadistance_right;
  using _state_left_type =
    uint8_t;
  _state_left_type state_left;
  using _state_right_type =
    uint8_t;
  _state_right_type state_right;
  using _current_left_type =
    int16_t;
  _current_left_type current_left;
  using _current_right_type =
    int16_t;
  _current_right_type current_right;
  using _speed_left_type =
    float;
  _speed_left_type speed_left;
  using _speed_right_type =
    float;
  _speed_right_type speed_right;
  using _isrunning_left_type =
    bool;
  _isrunning_left_type isrunning_left;
  using _isrunning_right_type =
    bool;
  _isrunning_right_type isrunning_right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__power_left(
    const uint16_t & _arg)
  {
    this->power_left = _arg;
    return *this;
  }
  Type & set__power_right(
    const uint16_t & _arg)
  {
    this->power_right = _arg;
    return *this;
  }
  Type & set__counter_left(
    const int32_t & _arg)
  {
    this->counter_left = _arg;
    return *this;
  }
  Type & set__counter_right(
    const int32_t & _arg)
  {
    this->counter_right = _arg;
    return *this;
  }
  Type & set__free_left(
    const uint8_t & _arg)
  {
    this->free_left = _arg;
    return *this;
  }
  Type & set__free_right(
    const uint8_t & _arg)
  {
    this->free_right = _arg;
    return *this;
  }
  Type & set__deltadistance_left(
    const float & _arg)
  {
    this->deltadistance_left = _arg;
    return *this;
  }
  Type & set__deltadistance_right(
    const float & _arg)
  {
    this->deltadistance_right = _arg;
    return *this;
  }
  Type & set__state_left(
    const uint8_t & _arg)
  {
    this->state_left = _arg;
    return *this;
  }
  Type & set__state_right(
    const uint8_t & _arg)
  {
    this->state_right = _arg;
    return *this;
  }
  Type & set__current_left(
    const int16_t & _arg)
  {
    this->current_left = _arg;
    return *this;
  }
  Type & set__current_right(
    const int16_t & _arg)
  {
    this->current_right = _arg;
    return *this;
  }
  Type & set__speed_left(
    const float & _arg)
  {
    this->speed_left = _arg;
    return *this;
  }
  Type & set__speed_right(
    const float & _arg)
  {
    this->speed_right = _arg;
    return *this;
  }
  Type & set__isrunning_left(
    const bool & _arg)
  {
    this->isrunning_left = _arg;
    return *this;
  }
  Type & set__isrunning_right(
    const bool & _arg)
  {
    this->isrunning_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::WheelPair_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::WheelPair_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::WheelPair_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::WheelPair_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::WheelPair_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::WheelPair_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::WheelPair_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::WheelPair_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::WheelPair_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::WheelPair_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__WheelPair
    std::shared_ptr<hqv_public_interface::msg::WheelPair_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__WheelPair
    std::shared_ptr<hqv_public_interface::msg::WheelPair_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WheelPair_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->power_left != other.power_left) {
      return false;
    }
    if (this->power_right != other.power_right) {
      return false;
    }
    if (this->counter_left != other.counter_left) {
      return false;
    }
    if (this->counter_right != other.counter_right) {
      return false;
    }
    if (this->free_left != other.free_left) {
      return false;
    }
    if (this->free_right != other.free_right) {
      return false;
    }
    if (this->deltadistance_left != other.deltadistance_left) {
      return false;
    }
    if (this->deltadistance_right != other.deltadistance_right) {
      return false;
    }
    if (this->state_left != other.state_left) {
      return false;
    }
    if (this->state_right != other.state_right) {
      return false;
    }
    if (this->current_left != other.current_left) {
      return false;
    }
    if (this->current_right != other.current_right) {
      return false;
    }
    if (this->speed_left != other.speed_left) {
      return false;
    }
    if (this->speed_right != other.speed_right) {
      return false;
    }
    if (this->isrunning_left != other.isrunning_left) {
      return false;
    }
    if (this->isrunning_right != other.isrunning_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const WheelPair_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WheelPair_

// alias to use template instance with default allocator
using WheelPair =
  hqv_public_interface::msg::WheelPair_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__STRUCT_HPP_
