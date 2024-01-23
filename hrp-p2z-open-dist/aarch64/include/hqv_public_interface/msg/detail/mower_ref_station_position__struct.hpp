// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MowerRefStationPosition __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerRefStationPosition __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerRefStationPosition_
{
  using Type = MowerRefStationPosition_<ContainerAllocator>;

  explicit MowerRefStationPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->longitude = 0l;
      this->latitude = 0l;
      this->altitude = 0l;
    }
  }

  explicit MowerRefStationPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->longitude = 0l;
      this->latitude = 0l;
      this->altitude = 0l;
    }
  }

  // field types and members
  using _longitude_type =
    int32_t;
  _longitude_type longitude;
  using _latitude_type =
    int32_t;
  _latitude_type latitude;
  using _altitude_type =
    int32_t;
  _altitude_type altitude;

  // setters for named parameter idiom
  Type & set__longitude(
    const int32_t & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__latitude(
    const int32_t & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const int32_t & _arg)
  {
    this->altitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerRefStationPosition
    std::shared_ptr<hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerRefStationPosition
    std::shared_ptr<hqv_public_interface::msg::MowerRefStationPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerRefStationPosition_ & other) const
  {
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerRefStationPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerRefStationPosition_

// alias to use template instance with default allocator
using MowerRefStationPosition =
  hqv_public_interface::msg::MowerRefStationPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_HPP_
