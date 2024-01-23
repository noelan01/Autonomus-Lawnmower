// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerSlamHandling.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MowerSlamHandling __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerSlamHandling __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerSlamHandling_
{
  using Type = MowerSlamHandling_<ContainerAllocator>;

  explicit MowerSlamHandling_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slam_handle = 0;
    }
  }

  explicit MowerSlamHandling_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->slam_handle = 0;
    }
  }

  // field types and members
  using _slam_handle_type =
    uint8_t;
  _slam_handle_type slam_handle;

  // setters for named parameter idiom
  Type & set__slam_handle(
    const uint8_t & _arg)
  {
    this->slam_handle = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SLAM_HANDLE_START =
    0u;
  static constexpr uint8_t SLAM_HANDLE_STOP =
    1u;
  static constexpr uint8_t SLAM_HANDLE_RESET =
    2u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerSlamHandling
    std::shared_ptr<hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerSlamHandling
    std::shared_ptr<hqv_public_interface::msg::MowerSlamHandling_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerSlamHandling_ & other) const
  {
    if (this->slam_handle != other.slam_handle) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerSlamHandling_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerSlamHandling_

// alias to use template instance with default allocator
using MowerSlamHandling =
  hqv_public_interface::msg::MowerSlamHandling_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamHandling_<ContainerAllocator>::SLAM_HANDLE_START;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamHandling_<ContainerAllocator>::SLAM_HANDLE_STOP;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamHandling_<ContainerAllocator>::SLAM_HANDLE_RESET;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__STRUCT_HPP_
