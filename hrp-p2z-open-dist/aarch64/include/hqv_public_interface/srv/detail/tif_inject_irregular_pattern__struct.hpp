// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/TifInjectIrregularPattern.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__TifInjectIrregularPattern_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__TifInjectIrregularPattern_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TifInjectIrregularPattern_Request_
{
  using Type = TifInjectIrregularPattern_Request_<ContainerAllocator>;

  explicit TifInjectIrregularPattern_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->set_state = 0;
    }
  }

  explicit TifInjectIrregularPattern_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->set_state = 0;
    }
  }

  // field types and members
  using _set_state_type =
    uint8_t;
  _set_state_type set_state;

  // setters for named parameter idiom
  Type & set__set_state(
    const uint8_t & _arg)
  {
    this->set_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__TifInjectIrregularPattern_Request
    std::shared_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__TifInjectIrregularPattern_Request
    std::shared_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TifInjectIrregularPattern_Request_ & other) const
  {
    if (this->set_state != other.set_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const TifInjectIrregularPattern_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TifInjectIrregularPattern_Request_

// alias to use template instance with default allocator
using TifInjectIrregularPattern_Request =
  hqv_public_interface::srv::TifInjectIrregularPattern_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__TifInjectIrregularPattern_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__TifInjectIrregularPattern_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TifInjectIrregularPattern_Response_
{
  using Type = TifInjectIrregularPattern_Response_<ContainerAllocator>;

  explicit TifInjectIrregularPattern_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit TifInjectIrregularPattern_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__TifInjectIrregularPattern_Response
    std::shared_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__TifInjectIrregularPattern_Response
    std::shared_ptr<hqv_public_interface::srv::TifInjectIrregularPattern_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TifInjectIrregularPattern_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const TifInjectIrregularPattern_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TifInjectIrregularPattern_Response_

// alias to use template instance with default allocator
using TifInjectIrregularPattern_Response =
  hqv_public_interface::srv::TifInjectIrregularPattern_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct TifInjectIrregularPattern
{
  using Request = hqv_public_interface::srv::TifInjectIrregularPattern_Request;
  using Response = hqv_public_interface::srv::TifInjectIrregularPattern_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__STRUCT_HPP_
