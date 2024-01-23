// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerGnssHeading.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HEADING__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HEADING__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerGnssHeading __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerGnssHeading __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerGnssHeading_
{
  using Type = MowerGnssHeading_<ContainerAllocator>;

  explicit MowerGnssHeading_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motion = 0.0;
      this->mower = 0.0;
      this->acc = 0.0;
    }
  }

  explicit MowerGnssHeading_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motion = 0.0;
      this->mower = 0.0;
      this->acc = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motion_type =
    double;
  _motion_type motion;
  using _mower_type =
    double;
  _mower_type mower;
  using _acc_type =
    double;
  _acc_type acc;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__motion(
    const double & _arg)
  {
    this->motion = _arg;
    return *this;
  }
  Type & set__mower(
    const double & _arg)
  {
    this->mower = _arg;
    return *this;
  }
  Type & set__acc(
    const double & _arg)
  {
    this->acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssHeading
    std::shared_ptr<hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssHeading
    std::shared_ptr<hqv_public_interface::msg::MowerGnssHeading_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerGnssHeading_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motion != other.motion) {
      return false;
    }
    if (this->mower != other.mower) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerGnssHeading_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerGnssHeading_

// alias to use template instance with default allocator
using MowerGnssHeading =
  hqv_public_interface::msg::MowerGnssHeading_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HEADING__STRUCT_HPP_
