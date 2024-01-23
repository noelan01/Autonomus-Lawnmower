// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerChargingStationStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CHARGING_STATION_STATUS__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CHARGING_STATION_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MowerChargingStationStatus __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerChargingStationStatus __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerChargingStationStatus_
{
  using Type = MowerChargingStationStatus_<ContainerAllocator>;

  explicit MowerChargingStationStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mower_in_cs = false;
    }
  }

  explicit MowerChargingStationStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mower_in_cs = false;
    }
  }

  // field types and members
  using _mower_in_cs_type =
    bool;
  _mower_in_cs_type mower_in_cs;

  // setters for named parameter idiom
  Type & set__mower_in_cs(
    const bool & _arg)
  {
    this->mower_in_cs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerChargingStationStatus
    std::shared_ptr<hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerChargingStationStatus
    std::shared_ptr<hqv_public_interface::msg::MowerChargingStationStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerChargingStationStatus_ & other) const
  {
    if (this->mower_in_cs != other.mower_in_cs) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerChargingStationStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerChargingStationStatus_

// alias to use template instance with default allocator
using MowerChargingStationStatus =
  hqv_public_interface::msg::MowerChargingStationStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CHARGING_STATION_STATUS__STRUCT_HPP_
