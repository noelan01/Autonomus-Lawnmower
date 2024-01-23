// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MapShape.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MapShape __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MapShape __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MapShape_
{
  using Type = MapShape_<ContainerAllocator>;

  explicit MapShape_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : polygon(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 64>::iterator, uint8_t>(this->id.begin(), this->id.end(), 0);
      this->type = 0;
      this->stroke_color = 0ul;
      this->fill_color = 0ul;
      this->hidden = false;
    }
  }

  explicit MapShape_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : polygon(_alloc, _init),
    id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint8_t, 64>::iterator, uint8_t>(this->id.begin(), this->id.end(), 0);
      this->type = 0;
      this->stroke_color = 0ul;
      this->fill_color = 0ul;
      this->hidden = false;
    }
  }

  // field types and members
  using _polygon_type =
    geometry_msgs::msg::Polygon_<ContainerAllocator>;
  _polygon_type polygon;
  using _id_type =
    std::array<uint8_t, 64>;
  _id_type id;
  using _type_type =
    int8_t;
  _type_type type;
  using _stroke_color_type =
    uint32_t;
  _stroke_color_type stroke_color;
  using _fill_color_type =
    uint32_t;
  _fill_color_type fill_color;
  using _hidden_type =
    bool;
  _hidden_type hidden;

  // setters for named parameter idiom
  Type & set__polygon(
    const geometry_msgs::msg::Polygon_<ContainerAllocator> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }
  Type & set__id(
    const std::array<uint8_t, 64> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__stroke_color(
    const uint32_t & _arg)
  {
    this->stroke_color = _arg;
    return *this;
  }
  Type & set__fill_color(
    const uint32_t & _arg)
  {
    this->fill_color = _arg;
    return *this;
  }
  Type & set__hidden(
    const bool & _arg)
  {
    this->hidden = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t UNDEFIED_AREA =
    -1;
  static constexpr int8_t WORKING_AREA =
    0;
  static constexpr int8_t TRANSPORT_AREA =
    1;
  static constexpr uint8_t FORBIDDEN_AREA =
    2u;
  static constexpr uint8_t HIDDEN_AREA =
    3u;
  static constexpr int8_t TRANSPORT_PATH =
    4;
  static constexpr int8_t POINTS_OF_INTEREST_PATH =
    5;
  static constexpr int8_t CHARGING_STATION_PATH =
    6;

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MapShape_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MapShape_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MapShape_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MapShape_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MapShape_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MapShape_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MapShape_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MapShape_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MapShape_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MapShape_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MapShape
    std::shared_ptr<hqv_public_interface::msg::MapShape_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MapShape
    std::shared_ptr<hqv_public_interface::msg::MapShape_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapShape_ & other) const
  {
    if (this->polygon != other.polygon) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->stroke_color != other.stroke_color) {
      return false;
    }
    if (this->fill_color != other.fill_color) {
      return false;
    }
    if (this->hidden != other.hidden) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapShape_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapShape_

// alias to use template instance with default allocator
using MapShape =
  hqv_public_interface::msg::MapShape_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t MapShape_<ContainerAllocator>::UNDEFIED_AREA;
template<typename ContainerAllocator>
constexpr int8_t MapShape_<ContainerAllocator>::WORKING_AREA;
template<typename ContainerAllocator>
constexpr int8_t MapShape_<ContainerAllocator>::TRANSPORT_AREA;
template<typename ContainerAllocator>
constexpr uint8_t MapShape_<ContainerAllocator>::FORBIDDEN_AREA;
template<typename ContainerAllocator>
constexpr uint8_t MapShape_<ContainerAllocator>::HIDDEN_AREA;
template<typename ContainerAllocator>
constexpr int8_t MapShape_<ContainerAllocator>::TRANSPORT_PATH;
template<typename ContainerAllocator>
constexpr int8_t MapShape_<ContainerAllocator>::POINTS_OF_INTEREST_PATH;
template<typename ContainerAllocator>
constexpr int8_t MapShape_<ContainerAllocator>::CHARGING_STATION_PATH;

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__STRUCT_HPP_
