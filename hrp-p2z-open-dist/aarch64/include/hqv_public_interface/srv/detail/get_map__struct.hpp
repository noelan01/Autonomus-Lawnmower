// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/GetMap.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__GetMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__GetMap_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMap_Request_
{
  using Type = GetMap_Request_<ContainerAllocator>;

  explicit GetMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    hqv_public_interface::srv::GetMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::GetMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::GetMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::GetMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::GetMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::GetMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::GetMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::GetMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::GetMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::GetMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__GetMap_Request
    std::shared_ptr<hqv_public_interface::srv::GetMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__GetMap_Request
    std::shared_ptr<hqv_public_interface::srv::GetMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMap_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMap_Request_

// alias to use template instance with default allocator
using GetMap_Request =
  hqv_public_interface::srv::GetMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface


// Include directives for member types
// Member 'map_info'
#include "hqv_public_interface/msg/detail/map_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__GetMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__GetMap_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMap_Response_
{
  using Type = GetMap_Response_<ContainerAllocator>;

  explicit GetMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_info(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0ll;
    }
  }

  explicit GetMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_info(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0ll;
    }
  }

  // field types and members
  using _result_type =
    int64_t;
  _result_type result;
  using _map_info_type =
    hqv_public_interface::msg::MapInfo_<ContainerAllocator>;
  _map_info_type map_info;

  // setters for named parameter idiom
  Type & set__result(
    const int64_t & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__map_info(
    const hqv_public_interface::msg::MapInfo_<ContainerAllocator> & _arg)
  {
    this->map_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::GetMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::GetMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::GetMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::GetMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::GetMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::GetMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::GetMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::GetMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::GetMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::GetMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__GetMap_Response
    std::shared_ptr<hqv_public_interface::srv::GetMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__GetMap_Response
    std::shared_ptr<hqv_public_interface::srv::GetMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMap_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->map_info != other.map_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMap_Response_

// alias to use template instance with default allocator
using GetMap_Response =
  hqv_public_interface::srv::GetMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct GetMap
{
  using Request = hqv_public_interface::srv::GetMap_Request;
  using Response = hqv_public_interface::srv::GetMap_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__STRUCT_HPP_
