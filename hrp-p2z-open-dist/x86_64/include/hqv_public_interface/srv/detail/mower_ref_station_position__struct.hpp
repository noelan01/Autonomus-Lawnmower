// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__MowerRefStationPosition_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__MowerRefStationPosition_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MowerRefStationPosition_Request_
{
  using Type = MowerRefStationPosition_Request_<ContainerAllocator>;

  explicit MowerRefStationPosition_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_type = 0;
    }
  }

  explicit MowerRefStationPosition_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_type = 0;
    }
  }

  // field types and members
  using _message_type_type =
    uint8_t;
  _message_type_type message_type;

  // setters for named parameter idiom
  Type & set__message_type(
    const uint8_t & _arg)
  {
    this->message_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MESSAGE_TYPE_INSTALLED_POSITION =
    0u;
  static constexpr uint8_t MESSAGE_TYPE_CURRENT_POSITION =
    1u;

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__MowerRefStationPosition_Request
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__MowerRefStationPosition_Request
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationPosition_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerRefStationPosition_Request_ & other) const
  {
    if (this->message_type != other.message_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerRefStationPosition_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerRefStationPosition_Request_

// alias to use template instance with default allocator
using MowerRefStationPosition_Request =
  hqv_public_interface::srv::MowerRefStationPosition_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerRefStationPosition_Request_<ContainerAllocator>::MESSAGE_TYPE_INSTALLED_POSITION;
template<typename ContainerAllocator>
constexpr uint8_t MowerRefStationPosition_Request_<ContainerAllocator>::MESSAGE_TYPE_CURRENT_POSITION;

}  // namespace srv

}  // namespace hqv_public_interface


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__MowerRefStationPosition_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__MowerRefStationPosition_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MowerRefStationPosition_Response_
{
  using Type = MowerRefStationPosition_Response_<ContainerAllocator>;

  explicit MowerRefStationPosition_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
      this->longitude = 0l;
      this->latitude = 0l;
      this->altitude = 0l;
    }
  }

  explicit MowerRefStationPosition_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
      this->longitude = 0l;
      this->latitude = 0l;
      this->altitude = 0l;
    }
  }

  // field types and members
  using _status_type =
    bool;
  _status_type status;
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
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }
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
    hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__MowerRefStationPosition_Response
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__MowerRefStationPosition_Response
    std::shared_ptr<hqv_public_interface::srv::MowerRefStationPosition_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerRefStationPosition_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
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
  bool operator!=(const MowerRefStationPosition_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerRefStationPosition_Response_

// alias to use template instance with default allocator
using MowerRefStationPosition_Response =
  hqv_public_interface::srv::MowerRefStationPosition_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct MowerRefStationPosition
{
  using Request = hqv_public_interface::srv::MowerRefStationPosition_Request;
  using Response = hqv_public_interface::srv::MowerRefStationPosition_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_HPP_
