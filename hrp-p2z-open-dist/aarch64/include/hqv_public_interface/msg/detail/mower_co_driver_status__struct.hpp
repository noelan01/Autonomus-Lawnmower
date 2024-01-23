// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_STATUS__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MowerCoDriverStatus __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerCoDriverStatus __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerCoDriverStatus_
{
  using Type = MowerCoDriverStatus_<ContainerAllocator>;

  explicit MowerCoDriverStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit MowerCoDriverStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ICODRIVER_STATUS_CLOSED =
    0u;
  static constexpr uint8_t ICODRIVER_STATUS_OPEN =
    1u;
  static constexpr uint8_t ICODRIVER_STATUS_UNKNOWN =
    2u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerCoDriverStatus
    std::shared_ptr<hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerCoDriverStatus
    std::shared_ptr<hqv_public_interface::msg::MowerCoDriverStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerCoDriverStatus_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerCoDriverStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerCoDriverStatus_

// alias to use template instance with default allocator
using MowerCoDriverStatus =
  hqv_public_interface::msg::MowerCoDriverStatus_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverStatus_<ContainerAllocator>::ICODRIVER_STATUS_CLOSED;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverStatus_<ContainerAllocator>::ICODRIVER_STATUS_OPEN;
template<typename ContainerAllocator>
constexpr uint8_t MowerCoDriverStatus_<ContainerAllocator>::ICODRIVER_STATUS_UNKNOWN;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_STATUS__STRUCT_HPP_
