// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:srv/GetSiteMap.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__GetSiteMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__GetSiteMap_Request __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSiteMap_Request_
{
  using Type = GetSiteMap_Request_<ContainerAllocator>;

  explicit GetSiteMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = 0;
    }
  }

  explicit GetSiteMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = 0;
    }
  }

  // field types and members
  using _request_type =
    uint8_t;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const uint8_t & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MAP_SITENAME =
    0u;
  static constexpr uint8_t MAP_DATA =
    1u;
  static constexpr uint8_t MAP_UPDATE =
    2u;

  // pointer types
  using RawPtr =
    hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__GetSiteMap_Request
    std::shared_ptr<hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__GetSiteMap_Request
    std::shared_ptr<hqv_public_interface::srv::GetSiteMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSiteMap_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSiteMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSiteMap_Request_

// alias to use template instance with default allocator
using GetSiteMap_Request =
  hqv_public_interface::srv::GetSiteMap_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t GetSiteMap_Request_<ContainerAllocator>::MAP_SITENAME;
template<typename ContainerAllocator>
constexpr uint8_t GetSiteMap_Request_<ContainerAllocator>::MAP_DATA;
template<typename ContainerAllocator>
constexpr uint8_t GetSiteMap_Request_<ContainerAllocator>::MAP_UPDATE;

}  // namespace srv

}  // namespace hqv_public_interface


#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__srv__GetSiteMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__srv__GetSiteMap_Response __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSiteMap_Response_
{
  using Type = GetSiteMap_Response_<ContainerAllocator>;

  explicit GetSiteMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->loaded = false;
      this->valid = false;
      this->sitename = "";
    }
  }

  explicit GetSiteMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sitename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->loaded = false;
      this->valid = false;
      this->sitename = "";
    }
  }

  // field types and members
  using _loaded_type =
    bool;
  _loaded_type loaded;
  using _valid_type =
    bool;
  _valid_type valid;
  using _sitename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sitename_type sitename;
  using _data_type =
    std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__loaded(
    const bool & _arg)
  {
    this->loaded = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__sitename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sitename = _arg;
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
    hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__srv__GetSiteMap_Response
    std::shared_ptr<hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__srv__GetSiteMap_Response
    std::shared_ptr<hqv_public_interface::srv::GetSiteMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSiteMap_Response_ & other) const
  {
    if (this->loaded != other.loaded) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->sitename != other.sitename) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSiteMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSiteMap_Response_

// alias to use template instance with default allocator
using GetSiteMap_Response =
  hqv_public_interface::srv::GetSiteMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hqv_public_interface

namespace hqv_public_interface
{

namespace srv
{

struct GetSiteMap
{
  using Request = hqv_public_interface::srv::GetSiteMap_Request;
  using Response = hqv_public_interface::srv::GetSiteMap_Response;
};

}  // namespace srv

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__STRUCT_HPP_
