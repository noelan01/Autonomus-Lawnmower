// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/MowerSlamMapHandling.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__MowerSlamMapHandling_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__MowerSlamMapHandling_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MowerSlamMapHandling_Request_
{
  using Type = MowerSlamMapHandling_Request_<ContainerAllocator>;

  explicit MowerSlamMapHandling_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_type = 0;
      this->map_id = 0ul;
    }
  }

  explicit MowerSlamMapHandling_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_type = 0;
      this->map_id = 0ul;
    }
  }

  // field types and members
  using _message_type_type =
    uint8_t;
  _message_type_type message_type;
  using _map_id_type =
    uint32_t;
  _map_id_type map_id;

  // setters for named parameter idiom
  Type & set__message_type(
    const uint8_t & _arg)
  {
    this->message_type = _arg;
    return *this;
  }
  Type & set__map_id(
    const uint32_t & _arg)
  {
    this->map_id = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MESSAGE_TYPE_GET_ACTIVE_MAP =
    1u;
  static constexpr uint8_t MESSAGE_TYPE_LIST_MAPS =
    4u;

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__MowerSlamMapHandling_Request
    std::shared_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__MowerSlamMapHandling_Request
    std::shared_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerSlamMapHandling_Request_ & other) const
  {
    if (this->message_type != other.message_type) {
      return false;
    }
    if (this->map_id != other.map_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerSlamMapHandling_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerSlamMapHandling_Request_

// alias to use template instance with default allocator
using MowerSlamMapHandling_Request =
  hqv_public_interface::srv::MowerSlamMapHandling_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamMapHandling_Request_<ContainerAllocator>::MESSAGE_TYPE_GET_ACTIVE_MAP;
template<typename ContainerAllocator>
constexpr uint8_t MowerSlamMapHandling_Request_<ContainerAllocator>::MESSAGE_TYPE_LIST_MAPS;

}  // namespace srv

}  // namespace hqv_public_interface


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__MowerSlamMapHandling_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__MowerSlamMapHandling_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MowerSlamMapHandling_Response_
{
  using Type = MowerSlamMapHandling_Response_<ContainerAllocator>;

  explicit MowerSlamMapHandling_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_id = 0ul;
      std::fill<typename std::array<uint32_t, 10>::iterator, uint32_t>(this->map_ids.begin(), this->map_ids.end(), 0ul);
    }
  }

  explicit MowerSlamMapHandling_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_ids(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_id = 0ul;
      std::fill<typename std::array<uint32_t, 10>::iterator, uint32_t>(this->map_ids.begin(), this->map_ids.end(), 0ul);
    }
  }

  // field types and members
  using _map_id_type =
    uint32_t;
  _map_id_type map_id;
  using _map_ids_type =
    std::array<uint32_t, 10>;
  _map_ids_type map_ids;

  // setters for named parameter idiom
  Type & set__map_id(
    const uint32_t & _arg)
  {
    this->map_id = _arg;
    return *this;
  }
  Type & set__map_ids(
    const std::array<uint32_t, 10> & _arg)
  {
    this->map_ids = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__MowerSlamMapHandling_Response
    std::shared_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__MowerSlamMapHandling_Response
    std::shared_ptr<hqv_public_interface::srv::MowerSlamMapHandling_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerSlamMapHandling_Response_ & other) const
  {
    if (this->map_id != other.map_id) {
      return false;
    }
    if (this->map_ids != other.map_ids) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerSlamMapHandling_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerSlamMapHandling_Response_

// alias to use template instance with default allocator
using MowerSlamMapHandling_Response =
  hqv_public_interface::srv::MowerSlamMapHandling_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct MowerSlamMapHandling
{
  using Request = hqv_public_interface::srv::MowerSlamMapHandling_Request;
  using Response = hqv_public_interface::srv::MowerSlamMapHandling_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__STRUCT_HPP_
