// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/TifGetFile.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__TifGetFile_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__TifGetFile_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TifGetFile_Request_
{
  using Type = TifGetFile_Request_<ContainerAllocator>;

  explicit TifGetFile_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->filename = "";
    }
  }

  explicit TifGetFile_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : filename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->filename = "";
    }
  }

  // field types and members
  using _filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _filename_type filename;

  // setters for named parameter idiom
  Type & set__filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->filename = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__TifGetFile_Request
    std::shared_ptr<hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__TifGetFile_Request
    std::shared_ptr<hqv_public_interface::srv::TifGetFile_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TifGetFile_Request_ & other) const
  {
    if (this->filename != other.filename) {
      return false;
    }
    return true;
  }
  bool operator!=(const TifGetFile_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TifGetFile_Request_

// alias to use template instance with default allocator
using TifGetFile_Request =
  hqv_public_interface::srv::TifGetFile_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__TifGetFile_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__TifGetFile_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TifGetFile_Response_
{
  using Type = TifGetFile_Response_<ContainerAllocator>;

  explicit TifGetFile_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit TifGetFile_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _data_type =
    std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
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
    hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__TifGetFile_Response
    std::shared_ptr<hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__TifGetFile_Response
    std::shared_ptr<hqv_public_interface::srv::TifGetFile_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TifGetFile_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const TifGetFile_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TifGetFile_Response_

// alias to use template instance with default allocator
using TifGetFile_Response =
  hqv_public_interface::srv::TifGetFile_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct TifGetFile
{
  using Request = hqv_public_interface::srv::TifGetFile_Request;
  using Response = hqv_public_interface::srv::TifGetFile_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__STRUCT_HPP_
