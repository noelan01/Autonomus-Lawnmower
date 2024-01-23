// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/HwMon.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__HwMon __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__HwMon __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HwMon_
{
  using Type = HwMon_<ContainerAllocator>;

  explicit HwMon_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->temperature = 0l;
      this->cpu_freq = 0l;
      this->used_memory = 0l;
    }
  }

  explicit HwMon_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->temperature = 0l;
      this->cpu_freq = 0l;
      this->used_memory = 0l;
    }
  }

  // field types and members
  using _temperature_type =
    int32_t;
  _temperature_type temperature;
  using _cpu_freq_type =
    int32_t;
  _cpu_freq_type cpu_freq;
  using _used_memory_type =
    int32_t;
  _used_memory_type used_memory;

  // setters for named parameter idiom
  Type & set__temperature(
    const int32_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__cpu_freq(
    const int32_t & _arg)
  {
    this->cpu_freq = _arg;
    return *this;
  }
  Type & set__used_memory(
    const int32_t & _arg)
  {
    this->used_memory = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::HwMon_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::HwMon_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::HwMon_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::HwMon_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::HwMon_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::HwMon_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::HwMon_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::HwMon_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::HwMon_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::HwMon_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__HwMon
    std::shared_ptr<hqv_public_interface::msg::HwMon_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__HwMon
    std::shared_ptr<hqv_public_interface::msg::HwMon_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HwMon_ & other) const
  {
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->cpu_freq != other.cpu_freq) {
      return false;
    }
    if (this->used_memory != other.used_memory) {
      return false;
    }
    return true;
  }
  bool operator!=(const HwMon_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HwMon_

// alias to use template instance with default allocator
using HwMon =
  hqv_public_interface::msg::HwMon_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__STRUCT_HPP_
