// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerLoopSignal.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LOOP_SIGNAL__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LOOP_SIGNAL__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerLoopSignal __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerLoopSignal __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerLoopSignal_
{
  using Type = MowerLoopSignal_<ContainerAllocator>;

  explicit MowerLoopSignal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->inside = false;
      this->a = 0;
      this->f = 0;
      this->n = 0;
    }
  }

  explicit MowerLoopSignal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0;
      this->inside = false;
      this->a = 0;
      this->f = 0;
      this->n = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _index_type =
    uint8_t;
  _index_type index;
  using _inside_type =
    bool;
  _inside_type inside;
  using _a_type =
    int16_t;
  _a_type a;
  using _f_type =
    int16_t;
  _f_type f;
  using _n_type =
    int16_t;
  _n_type n;

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
  Type & set__inside(
    const bool & _arg)
  {
    this->inside = _arg;
    return *this;
  }
  Type & set__a(
    const int16_t & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__f(
    const int16_t & _arg)
  {
    this->f = _arg;
    return *this;
  }
  Type & set__n(
    const int16_t & _arg)
  {
    this->n = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerLoopSignal
    std::shared_ptr<hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerLoopSignal
    std::shared_ptr<hqv_public_interface::msg::MowerLoopSignal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerLoopSignal_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->inside != other.inside) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    if (this->f != other.f) {
      return false;
    }
    if (this->n != other.n) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerLoopSignal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerLoopSignal_

// alias to use template instance with default allocator
using MowerLoopSignal =
  hqv_public_interface::msg::MowerLoopSignal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LOOP_SIGNAL__STRUCT_HPP_
