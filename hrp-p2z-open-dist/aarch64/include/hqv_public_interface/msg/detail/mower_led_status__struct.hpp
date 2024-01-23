// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerLedStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerLedStatus __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerLedStatus __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerLedStatus_
{
  using Type = MowerLedStatus_<ContainerAllocator>;

  explicit MowerLedStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mower_spp_state = 0;
      this->mower_app_activity = 0;
      this->extended_status_host_message = 0;
      this->operation_manager_state = 0;
      this->in_charging_station = false;
      this->is_stopped = false;
      this->color = 0ul;
      this->led_mode = 0;
      this->period = 0;
      this->log_id = 0;
    }
  }

  explicit MowerLedStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mower_spp_state = 0;
      this->mower_app_activity = 0;
      this->extended_status_host_message = 0;
      this->operation_manager_state = 0;
      this->in_charging_station = false;
      this->is_stopped = false;
      this->color = 0ul;
      this->led_mode = 0;
      this->period = 0;
      this->log_id = 0;
    }
  }

  // field types and members
  using _mower_spp_state_type =
    uint8_t;
  _mower_spp_state_type mower_spp_state;
  using _mower_app_activity_type =
    uint8_t;
  _mower_app_activity_type mower_app_activity;
  using _extended_status_host_message_type =
    uint8_t;
  _extended_status_host_message_type extended_status_host_message;
  using _operation_manager_state_type =
    uint8_t;
  _operation_manager_state_type operation_manager_state;
  using _in_charging_station_type =
    bool;
  _in_charging_station_type in_charging_station;
  using _is_stopped_type =
    bool;
  _is_stopped_type is_stopped;
  using _color_type =
    uint32_t;
  _color_type color;
  using _led_mode_type =
    uint8_t;
  _led_mode_type led_mode;
  using _period_type =
    uint16_t;
  _period_type period;
  using _log_id_type =
    uint8_t;
  _log_id_type log_id;
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;

  // setters for named parameter idiom
  Type & set__mower_spp_state(
    const uint8_t & _arg)
  {
    this->mower_spp_state = _arg;
    return *this;
  }
  Type & set__mower_app_activity(
    const uint8_t & _arg)
  {
    this->mower_app_activity = _arg;
    return *this;
  }
  Type & set__extended_status_host_message(
    const uint8_t & _arg)
  {
    this->extended_status_host_message = _arg;
    return *this;
  }
  Type & set__operation_manager_state(
    const uint8_t & _arg)
  {
    this->operation_manager_state = _arg;
    return *this;
  }
  Type & set__in_charging_station(
    const bool & _arg)
  {
    this->in_charging_station = _arg;
    return *this;
  }
  Type & set__is_stopped(
    const bool & _arg)
  {
    this->is_stopped = _arg;
    return *this;
  }
  Type & set__color(
    const uint32_t & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__led_mode(
    const uint8_t & _arg)
  {
    this->led_mode = _arg;
    return *this;
  }
  Type & set__period(
    const uint16_t & _arg)
  {
    this->period = _arg;
    return *this;
  }
  Type & set__log_id(
    const uint8_t & _arg)
  {
    this->log_id = _arg;
    return *this;
  }
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerLedStatus
    std::shared_ptr<hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerLedStatus
    std::shared_ptr<hqv_public_interface::msg::MowerLedStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerLedStatus_ & other) const
  {
    if (this->mower_spp_state != other.mower_spp_state) {
      return false;
    }
    if (this->mower_app_activity != other.mower_app_activity) {
      return false;
    }
    if (this->extended_status_host_message != other.extended_status_host_message) {
      return false;
    }
    if (this->operation_manager_state != other.operation_manager_state) {
      return false;
    }
    if (this->in_charging_station != other.in_charging_station) {
      return false;
    }
    if (this->is_stopped != other.is_stopped) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    if (this->led_mode != other.led_mode) {
      return false;
    }
    if (this->period != other.period) {
      return false;
    }
    if (this->log_id != other.log_id) {
      return false;
    }
    if (this->header != other.header) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerLedStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerLedStatus_

// alias to use template instance with default allocator
using MowerLedStatus =
  hqv_public_interface::msg::MowerLedStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__STRUCT_HPP_
