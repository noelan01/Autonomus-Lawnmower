// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerSlamFeedback.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MowerSlamFeedback __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerSlamFeedback __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerSlamFeedback_
{
  using Type = MowerSlamFeedback_<ContainerAllocator>;

  explicit MowerSlamFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback = 0;
      this->feedback_data_1 = 0l;
      this->feedback_data_2 = 0l;
    }
  }

  explicit MowerSlamFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback = 0;
      this->feedback_data_1 = 0l;
      this->feedback_data_2 = 0l;
    }
  }

  // field types and members
  using _feedback_type =
    uint8_t;
  _feedback_type feedback;
  using _feedback_data_1_type =
    int32_t;
  _feedback_data_1_type feedback_data_1;
  using _feedback_data_2_type =
    int32_t;
  _feedback_data_2_type feedback_data_2;

  // setters for named parameter idiom
  Type & set__feedback(
    const uint8_t & _arg)
  {
    this->feedback = _arg;
    return *this;
  }
  Type & set__feedback_data_1(
    const int32_t & _arg)
  {
    this->feedback_data_1 = _arg;
    return *this;
  }
  Type & set__feedback_data_2(
    const int32_t & _arg)
  {
    this->feedback_data_2 = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FEEDBACK_NORMAL =
    0u;
  static constexpr uint8_t FEEDBACK_FULL_STOP =
    1u;
  static constexpr uint8_t FEEDBACK_TURN_IN_PLACE =
    2u;
  static constexpr uint8_t FEEDBACK_ADAPT_MAX_VELOCITY =
    3u;
  static constexpr uint8_t FEEDBACK_CHANGE_HEADING =
    4u;
  static constexpr uint8_t FEEDBACK_DRIVE_STRAIGHT =
    5u;
  static constexpr uint8_t FEEDBACK_DRIVE_TO =
    6u;
  static constexpr uint8_t FEEDBACK_CALIBRATION_DONE =
    7u;
  static constexpr uint8_t FEEDBACK_RTK_ANOMALY =
    8u;
  static constexpr uint8_t FEEDBACK_UNKNOWN =
    9u;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerSlamFeedback
    std::shared_ptr<hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerSlamFeedback
    std::shared_ptr<hqv_public_interface::msg::MowerSlamFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerSlamFeedback_ & other) const
  {
    if (this->feedback != other.feedback) {
      return false;
    }
    if (this->feedback_data_1 != other.feedback_data_1) {
      return false;
    }
    if (this->feedback_data_2 != other.feedback_data_2) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerSlamFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerSlamFeedback_

// alias to use template instance with default allocator
using MowerSlamFeedback =
  hqv_public_interface::msg::MowerSlamFeedback_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_NORMAL;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_FULL_STOP;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_TURN_IN_PLACE;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_ADAPT_MAX_VELOCITY;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_CHANGE_HEADING;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_DRIVE_STRAIGHT;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_DRIVE_TO;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_CALIBRATION_DONE;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_RTK_ANOMALY;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamFeedback_<ContainerAllocator>::FEEDBACK_UNKNOWN;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__STRUCT_HPP_
