// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerGuideSignal.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerGuideSignal __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerGuideSignal __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerGuideSignal_
{
  using Type = MowerGuideSignal_<ContainerAllocator>;

  explicit MowerGuideSignal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->quality = 0;
      this->g1 = 0;
      this->g2 = 0;
      this->g3 = 0;
    }
  }

  explicit MowerGuideSignal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->quality = 0;
      this->g1 = 0;
      this->g2 = 0;
      this->g3 = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _index_type =
    uint8_t;
  _index_type index;
  using _quality_type =
    uint8_t;
  _quality_type quality;
  using _g1_type =
    int16_t;
  _g1_type g1;
  using _g2_type =
    int16_t;
  _g2_type g2;
  using _g3_type =
    int16_t;
  _g3_type g3;

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
  Type & set__quality(
    const uint8_t & _arg)
  {
    this->quality = _arg;
    return *this;
  }
  Type & set__g1(
    const int16_t & _arg)
  {
    this->g1 = _arg;
    return *this;
  }
  Type & set__g2(
    const int16_t & _arg)
  {
    this->g2 = _arg;
    return *this;
  }
  Type & set__g3(
    const int16_t & _arg)
  {
    this->g3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerGuideSignal
    std::shared_ptr<hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerGuideSignal
    std::shared_ptr<hqv_public_interface::msg::MowerGuideSignal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerGuideSignal_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->quality != other.quality) {
      return false;
    }
    if (this->g1 != other.g1) {
      return false;
    }
    if (this->g2 != other.g2) {
      return false;
    }
    if (this->g3 != other.g3) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerGuideSignal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerGuideSignal_

// alias to use template instance with default allocator
using MowerGuideSignal =
  hqv_public_interface::msg::MowerGuideSignal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__STRUCT_HPP_
