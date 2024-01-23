// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelativePositionENU.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerGnssRtkRelativePositionENU __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerGnssRtkRelativePositionENU __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerGnssRtkRelativePositionENU_
{
  using Type = MowerGnssRtkRelativePositionENU_<ContainerAllocator>;

  explicit MowerGnssRtkRelativePositionENU_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->east = 0.0;
      this->north = 0.0;
      this->up = 0.0;
      this->accuracy_east = 0.0f;
      this->accuracy_north = 0.0f;
      this->accuracy_up = 0.0f;
    }
  }

  explicit MowerGnssRtkRelativePositionENU_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->east = 0.0;
      this->north = 0.0;
      this->up = 0.0;
      this->accuracy_east = 0.0f;
      this->accuracy_north = 0.0f;
      this->accuracy_up = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _east_type =
    double;
  _east_type east;
  using _north_type =
    double;
  _north_type north;
  using _up_type =
    double;
  _up_type up;
  using _accuracy_east_type =
    float;
  _accuracy_east_type accuracy_east;
  using _accuracy_north_type =
    float;
  _accuracy_north_type accuracy_north;
  using _accuracy_up_type =
    float;
  _accuracy_up_type accuracy_up;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__east(
    const double & _arg)
  {
    this->east = _arg;
    return *this;
  }
  Type & set__north(
    const double & _arg)
  {
    this->north = _arg;
    return *this;
  }
  Type & set__up(
    const double & _arg)
  {
    this->up = _arg;
    return *this;
  }
  Type & set__accuracy_east(
    const float & _arg)
  {
    this->accuracy_east = _arg;
    return *this;
  }
  Type & set__accuracy_north(
    const float & _arg)
  {
    this->accuracy_north = _arg;
    return *this;
  }
  Type & set__accuracy_up(
    const float & _arg)
  {
    this->accuracy_up = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssRtkRelativePositionENU
    std::shared_ptr<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssRtkRelativePositionENU
    std::shared_ptr<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerGnssRtkRelativePositionENU_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->east != other.east) {
      return false;
    }
    if (this->north != other.north) {
      return false;
    }
    if (this->up != other.up) {
      return false;
    }
    if (this->accuracy_east != other.accuracy_east) {
      return false;
    }
    if (this->accuracy_north != other.accuracy_north) {
      return false;
    }
    if (this->accuracy_up != other.accuracy_up) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerGnssRtkRelativePositionENU_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerGnssRtkRelativePositionENU_

// alias to use template instance with default allocator
using MowerGnssRtkRelativePositionENU =
  hqv_public_interface::msg::MowerGnssRtkRelativePositionENU_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__STRUCT_HPP_
