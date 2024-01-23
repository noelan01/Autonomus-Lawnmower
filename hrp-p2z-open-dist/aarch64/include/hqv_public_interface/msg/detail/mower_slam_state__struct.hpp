// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerSlamState.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_STATE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MowerSlamState __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerSlamState __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerSlamState_
{
  using Type = MowerSlamState_<ContainerAllocator>;

  explicit MowerSlamState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit MowerSlamState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _state_type =
    uint8_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATE_RUNNING =
    0u;
  static constexpr uint8_t STATE_IDLE =
    1u;
  static constexpr uint8_t STATE_STARTED =
    2u;
  static constexpr uint8_t STATE_CONNECTED =
    3u;
  static constexpr uint8_t STATE_READY =
    4u;
  static constexpr uint8_t STATE_CALIBRATING =
    5u;
  static constexpr uint8_t STATE_FILTER_WARM_UP =
    6u;
  static constexpr uint8_t STATE_WAITING_FOR_GLOBAL_POS =
    7u;
  static constexpr uint8_t STATE_UNKNOWN =
    8u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerSlamState_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerSlamState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerSlamState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerSlamState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerSlamState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerSlamState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerSlamState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerSlamState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerSlamState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerSlamState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerSlamState
    std::shared_ptr<hqv_public_interface::msg::MowerSlamState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerSlamState
    std::shared_ptr<hqv_public_interface::msg::MowerSlamState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerSlamState_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerSlamState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerSlamState_

// alias to use template instance with default allocator
using MowerSlamState =
  hqv_public_interface::msg::MowerSlamState_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_RUNNING;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_IDLE;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_STARTED;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_CONNECTED;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_READY;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_CALIBRATING;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_FILTER_WARM_UP;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_WAITING_FOR_GLOBAL_POS;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamState_<ContainerAllocator>::STATE_UNKNOWN;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_STATE__STRUCT_HPP_
