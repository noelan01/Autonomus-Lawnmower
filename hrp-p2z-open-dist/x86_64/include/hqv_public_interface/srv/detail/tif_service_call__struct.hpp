// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/TifServiceCall.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__TifServiceCall_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__TifServiceCall_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TifServiceCall_Request_
{
  using Type = TifServiceCall_Request_<ContainerAllocator>;

  explicit TifServiceCall_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nodename = "";
      this->family = 0;
      this->command = 0;
      this->timeout = 0;
    }
  }

  explicit TifServiceCall_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : nodename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->nodename = "";
      this->family = 0;
      this->command = 0;
      this->timeout = 0;
    }
  }

  // field types and members
  using _nodename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _nodename_type nodename;
  using _family_type =
    uint16_t;
  _family_type family;
  using _command_type =
    uint16_t;
  _command_type command;
  using _payload_type =
    std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>>;
  _payload_type payload;
  using _timeout_type =
    uint16_t;
  _timeout_type timeout;

  // setters for named parameter idiom
  Type & set__nodename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->nodename = _arg;
    return *this;
  }
  Type & set__family(
    const uint16_t & _arg)
  {
    this->family = _arg;
    return *this;
  }
  Type & set__command(
    const uint16_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__payload(
    const std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>> & _arg)
  {
    this->payload = _arg;
    return *this;
  }
  Type & set__timeout(
    const uint16_t & _arg)
  {
    this->timeout = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__TifServiceCall_Request
    std::shared_ptr<hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__TifServiceCall_Request
    std::shared_ptr<hqv_public_interface::srv::TifServiceCall_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TifServiceCall_Request_ & other) const
  {
    if (this->nodename != other.nodename) {
      return false;
    }
    if (this->family != other.family) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->payload != other.payload) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    return true;
  }
  bool operator!=(const TifServiceCall_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TifServiceCall_Request_

// alias to use template instance with default allocator
using TifServiceCall_Request =
  hqv_public_interface::srv::TifServiceCall_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__TifServiceCall_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__TifServiceCall_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TifServiceCall_Response_
{
  using Type = TifServiceCall_Response_<ContainerAllocator>;

  explicit TifServiceCall_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
    }
  }

  explicit TifServiceCall_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0;
    }
  }

  // field types and members
  using _result_type =
    uint8_t;
  _result_type result;
  using _data_type =
    std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__result(
    const uint8_t & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__TifServiceCall_Response
    std::shared_ptr<hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__TifServiceCall_Response
    std::shared_ptr<hqv_public_interface::srv::TifServiceCall_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TifServiceCall_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const TifServiceCall_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TifServiceCall_Response_

// alias to use template instance with default allocator
using TifServiceCall_Response =
  hqv_public_interface::srv::TifServiceCall_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct TifServiceCall
{
  using Request = hqv_public_interface::srv::TifServiceCall_Request;
  using Response = hqv_public_interface::srv::TifServiceCall_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__STRUCT_HPP_
