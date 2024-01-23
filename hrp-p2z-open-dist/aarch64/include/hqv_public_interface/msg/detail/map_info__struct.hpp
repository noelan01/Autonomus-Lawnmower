// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MapInfo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'cs_pos'
// Member 'pc_pos'
#include "geometry_msgs/msg/detail/point32__struct.hpp"
// Member 'charging_station'
// Member 'working_areas'
// Member 'transport_areas'
// Member 'forbidden_areas'
// Member 'hidden_areas'
// Member 'transport_paths'
// Member 'poi_paths'
#include "hqv_public_interface/msg/detail/map_shape__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MapInfo __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MapInfo __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MapInfo_
{
  using Type = MapInfo_<ContainerAllocator>;

  explicit MapInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cs_pos(_init),
    pc_pos(_init),
    charging_station(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_name = "";
    }
  }

  explicit MapInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_name(_alloc),
    cs_pos(_alloc, _init),
    pc_pos(_alloc, _init),
    charging_station(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_name = "";
    }
  }

  // field types and members
  using _map_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _map_name_type map_name;
  using _cs_pos_type =
    geometry_msgs::msg::Point32_<ContainerAllocator>;
  _cs_pos_type cs_pos;
  using _pc_pos_type =
    geometry_msgs::msg::Point32_<ContainerAllocator>;
  _pc_pos_type pc_pos;
  using _charging_station_type =
    hqv_public_interface::msg::MapShape_<ContainerAllocator>;
  _charging_station_type charging_station;
  using _working_areas_type =
    std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>>;
  _working_areas_type working_areas;
  using _transport_areas_type =
    std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>>;
  _transport_areas_type transport_areas;
  using _forbidden_areas_type =
    std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>>;
  _forbidden_areas_type forbidden_areas;
  using _hidden_areas_type =
    std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>>;
  _hidden_areas_type hidden_areas;
  using _transport_paths_type =
    std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>>;
  _transport_paths_type transport_paths;
  using _poi_paths_type =
    std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>>;
  _poi_paths_type poi_paths;

  // setters for named parameter idiom
  Type & set__map_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->map_name = _arg;
    return *this;
  }
  Type & set__cs_pos(
    const geometry_msgs::msg::Point32_<ContainerAllocator> & _arg)
  {
    this->cs_pos = _arg;
    return *this;
  }
  Type & set__pc_pos(
    const geometry_msgs::msg::Point32_<ContainerAllocator> & _arg)
  {
    this->pc_pos = _arg;
    return *this;
  }
  Type & set__charging_station(
    const hqv_public_interface::msg::MapShape_<ContainerAllocator> & _arg)
  {
    this->charging_station = _arg;
    return *this;
  }
  Type & set__working_areas(
    const std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>> & _arg)
  {
    this->working_areas = _arg;
    return *this;
  }
  Type & set__transport_areas(
    const std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>> & _arg)
  {
    this->transport_areas = _arg;
    return *this;
  }
  Type & set__forbidden_areas(
    const std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>> & _arg)
  {
    this->forbidden_areas = _arg;
    return *this;
  }
  Type & set__hidden_areas(
    const std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>> & _arg)
  {
    this->hidden_areas = _arg;
    return *this;
  }
  Type & set__transport_paths(
    const std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>> & _arg)
  {
    this->transport_paths = _arg;
    return *this;
  }
  Type & set__poi_paths(
    const std::vector<hqv_public_interface::msg::MapShape_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hqv_public_interface::msg::MapShape_<ContainerAllocator>>> & _arg)
  {
    this->poi_paths = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MapInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MapInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MapInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MapInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MapInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MapInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MapInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MapInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MapInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MapInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MapInfo
    std::shared_ptr<hqv_public_interface::msg::MapInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MapInfo
    std::shared_ptr<hqv_public_interface::msg::MapInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapInfo_ & other) const
  {
    if (this->map_name != other.map_name) {
      return false;
    }
    if (this->cs_pos != other.cs_pos) {
      return false;
    }
    if (this->pc_pos != other.pc_pos) {
      return false;
    }
    if (this->charging_station != other.charging_station) {
      return false;
    }
    if (this->working_areas != other.working_areas) {
      return false;
    }
    if (this->transport_areas != other.transport_areas) {
      return false;
    }
    if (this->forbidden_areas != other.forbidden_areas) {
      return false;
    }
    if (this->hidden_areas != other.hidden_areas) {
      return false;
    }
    if (this->transport_paths != other.transport_paths) {
      return false;
    }
    if (this->poi_paths != other.poi_paths) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapInfo_

// alias to use template instance with default allocator
using MapInfo =
  hqv_public_interface::msg::MapInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__STRUCT_HPP_
