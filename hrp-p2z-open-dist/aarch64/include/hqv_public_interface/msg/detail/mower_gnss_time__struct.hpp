// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerGnssTime.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerGnssTime __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerGnssTime __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerGnssTime_
{
  using Type = MowerGnssTime_<ContainerAllocator>;

  explicit MowerGnssTime_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->minute = 0;
      this->second = 0;
      this->nano = 0l;
      this->timeaccuracy = 0ul;
      this->timevalid = false;
      this->datevalid = false;
    }
  }

  explicit MowerGnssTime_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->itow = 0ul;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->minute = 0;
      this->second = 0;
      this->nano = 0l;
      this->timeaccuracy = 0ul;
      this->timevalid = false;
      this->datevalid = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _itow_type =
    uint32_t;
  _itow_type itow;
  using _year_type =
    uint16_t;
  _year_type year;
  using _month_type =
    uint8_t;
  _month_type month;
  using _day_type =
    uint8_t;
  _day_type day;
  using _hour_type =
    uint8_t;
  _hour_type hour;
  using _minute_type =
    uint8_t;
  _minute_type minute;
  using _second_type =
    uint8_t;
  _second_type second;
  using _nano_type =
    int32_t;
  _nano_type nano;
  using _timeaccuracy_type =
    uint32_t;
  _timeaccuracy_type timeaccuracy;
  using _timevalid_type =
    bool;
  _timevalid_type timevalid;
  using _datevalid_type =
    bool;
  _datevalid_type datevalid;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__itow(
    const uint32_t & _arg)
  {
    this->itow = _arg;
    return *this;
  }
  Type & set__year(
    const uint16_t & _arg)
  {
    this->year = _arg;
    return *this;
  }
  Type & set__month(
    const uint8_t & _arg)
  {
    this->month = _arg;
    return *this;
  }
  Type & set__day(
    const uint8_t & _arg)
  {
    this->day = _arg;
    return *this;
  }
  Type & set__hour(
    const uint8_t & _arg)
  {
    this->hour = _arg;
    return *this;
  }
  Type & set__minute(
    const uint8_t & _arg)
  {
    this->minute = _arg;
    return *this;
  }
  Type & set__second(
    const uint8_t & _arg)
  {
    this->second = _arg;
    return *this;
  }
  Type & set__nano(
    const int32_t & _arg)
  {
    this->nano = _arg;
    return *this;
  }
  Type & set__timeaccuracy(
    const uint32_t & _arg)
  {
    this->timeaccuracy = _arg;
    return *this;
  }
  Type & set__timevalid(
    const bool & _arg)
  {
    this->timevalid = _arg;
    return *this;
  }
  Type & set__datevalid(
    const bool & _arg)
  {
    this->datevalid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssTime
    std::shared_ptr<hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssTime
    std::shared_ptr<hqv_public_interface::msg::MowerGnssTime_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerGnssTime_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->itow != other.itow) {
      return false;
    }
    if (this->year != other.year) {
      return false;
    }
    if (this->month != other.month) {
      return false;
    }
    if (this->day != other.day) {
      return false;
    }
    if (this->hour != other.hour) {
      return false;
    }
    if (this->minute != other.minute) {
      return false;
    }
    if (this->second != other.second) {
      return false;
    }
    if (this->nano != other.nano) {
      return false;
    }
    if (this->timeaccuracy != other.timeaccuracy) {
      return false;
    }
    if (this->timevalid != other.timevalid) {
      return false;
    }
    if (this->datevalid != other.datevalid) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerGnssTime_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerGnssTime_

// alias to use template instance with default allocator
using MowerGnssTime =
  hqv_public_interface::msg::MowerGnssTime_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__STRUCT_HPP_
