// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/HwMonService.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__HwMonService_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__HwMonService_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct HwMonService_Request_
{
  using Type = HwMonService_Request_<ContainerAllocator>;

  explicit HwMonService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit HwMonService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__HwMonService_Request
    std::shared_ptr<hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__HwMonService_Request
    std::shared_ptr<hqv_public_interface::srv::HwMonService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HwMonService_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const HwMonService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HwMonService_Request_

// alias to use template instance with default allocator
using HwMonService_Request =
  hqv_public_interface::srv::HwMonService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface


// Include directives for member types
// Member 'hw_monitor'
#include "hqv_public_interface/msg/detail/hw_mon__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__HwMonService_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__HwMonService_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct HwMonService_Response_
{
  using Type = HwMonService_Response_<ContainerAllocator>;

  explicit HwMonService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : hw_monitor(_init)
  {
    (void)_init;
  }

  explicit HwMonService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : hw_monitor(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _hw_monitor_type =
    hqv_public_interface::msg::HwMon_<ContainerAllocator>;
  _hw_monitor_type hw_monitor;

  // setters for named parameter idiom
  Type & set__hw_monitor(
    const hqv_public_interface::msg::HwMon_<ContainerAllocator> & _arg)
  {
    this->hw_monitor = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__HwMonService_Response
    std::shared_ptr<hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__HwMonService_Response
    std::shared_ptr<hqv_public_interface::srv::HwMonService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HwMonService_Response_ & other) const
  {
    if (this->hw_monitor != other.hw_monitor) {
      return false;
    }
    return true;
  }
  bool operator!=(const HwMonService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HwMonService_Response_

// alias to use template instance with default allocator
using HwMonService_Response =
  hqv_public_interface::srv::HwMonService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct HwMonService
{
  using Request = hqv_public_interface::srv::HwMonService_Request;
  using Response = hqv_public_interface::srv::HwMonService_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__STRUCT_HPP_
