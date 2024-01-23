// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelPosNEDRaw.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerGnssRtkRelPosNEDRaw_
{
  using Type = MowerGnssRtkRelPosNEDRaw_<ContainerAllocator>;

  explicit MowerGnssRtkRelPosNEDRaw_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_ms = 0ull;
      this->version = 0;
      this->reserved0 = 0;
      this->ref_station_id = 0;
      this->i_tow = 0ul;
      this->rel_pos_n = 0l;
      this->rel_pos_e = 0l;
      this->rel_pos_d = 0l;
      this->rel_pos_length = 0l;
      this->rel_pos_heading = 0l;
      this->reserved1_0 = 0;
      this->reserved1_1 = 0;
      this->reserved1_2 = 0;
      this->reserved1_3 = 0;
      this->rel_pos_hpn = 0;
      this->rel_pos_hpe = 0;
      this->rel_pos_hpd = 0;
      this->rel_pos_hp_length = 0;
      this->acc_n = 0ul;
      this->acc_e = 0ul;
      this->acc_d = 0ul;
      this->acc_length = 0ul;
      this->acc_heading = 0ul;
      this->reserved2_0 = 0;
      this->reserved2_1 = 0;
      this->reserved2_2 = 0;
      this->reserved2_3 = 0;
      this->flags = 0ul;
    }
  }

  explicit MowerGnssRtkRelPosNEDRaw_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_ms = 0ull;
      this->version = 0;
      this->reserved0 = 0;
      this->ref_station_id = 0;
      this->i_tow = 0ul;
      this->rel_pos_n = 0l;
      this->rel_pos_e = 0l;
      this->rel_pos_d = 0l;
      this->rel_pos_length = 0l;
      this->rel_pos_heading = 0l;
      this->reserved1_0 = 0;
      this->reserved1_1 = 0;
      this->reserved1_2 = 0;
      this->reserved1_3 = 0;
      this->rel_pos_hpn = 0;
      this->rel_pos_hpe = 0;
      this->rel_pos_hpd = 0;
      this->rel_pos_hp_length = 0;
      this->acc_n = 0ul;
      this->acc_e = 0ul;
      this->acc_d = 0ul;
      this->acc_length = 0ul;
      this->acc_heading = 0ul;
      this->reserved2_0 = 0;
      this->reserved2_1 = 0;
      this->reserved2_2 = 0;
      this->reserved2_3 = 0;
      this->flags = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_ms_type =
    uint64_t;
  _time_ms_type time_ms;
  using _version_type =
    uint8_t;
  _version_type version;
  using _reserved0_type =
    uint8_t;
  _reserved0_type reserved0;
  using _ref_station_id_type =
    uint16_t;
  _ref_station_id_type ref_station_id;
  using _i_tow_type =
    uint32_t;
  _i_tow_type i_tow;
  using _rel_pos_n_type =
    int32_t;
  _rel_pos_n_type rel_pos_n;
  using _rel_pos_e_type =
    int32_t;
  _rel_pos_e_type rel_pos_e;
  using _rel_pos_d_type =
    int32_t;
  _rel_pos_d_type rel_pos_d;
  using _rel_pos_length_type =
    int32_t;
  _rel_pos_length_type rel_pos_length;
  using _rel_pos_heading_type =
    int32_t;
  _rel_pos_heading_type rel_pos_heading;
  using _reserved1_0_type =
    uint8_t;
  _reserved1_0_type reserved1_0;
  using _reserved1_1_type =
    uint8_t;
  _reserved1_1_type reserved1_1;
  using _reserved1_2_type =
    uint8_t;
  _reserved1_2_type reserved1_2;
  using _reserved1_3_type =
    uint8_t;
  _reserved1_3_type reserved1_3;
  using _rel_pos_hpn_type =
    int8_t;
  _rel_pos_hpn_type rel_pos_hpn;
  using _rel_pos_hpe_type =
    int8_t;
  _rel_pos_hpe_type rel_pos_hpe;
  using _rel_pos_hpd_type =
    int8_t;
  _rel_pos_hpd_type rel_pos_hpd;
  using _rel_pos_hp_length_type =
    int8_t;
  _rel_pos_hp_length_type rel_pos_hp_length;
  using _acc_n_type =
    uint32_t;
  _acc_n_type acc_n;
  using _acc_e_type =
    uint32_t;
  _acc_e_type acc_e;
  using _acc_d_type =
    uint32_t;
  _acc_d_type acc_d;
  using _acc_length_type =
    uint32_t;
  _acc_length_type acc_length;
  using _acc_heading_type =
    uint32_t;
  _acc_heading_type acc_heading;
  using _reserved2_0_type =
    uint8_t;
  _reserved2_0_type reserved2_0;
  using _reserved2_1_type =
    uint8_t;
  _reserved2_1_type reserved2_1;
  using _reserved2_2_type =
    uint8_t;
  _reserved2_2_type reserved2_2;
  using _reserved2_3_type =
    uint8_t;
  _reserved2_3_type reserved2_3;
  using _flags_type =
    uint32_t;
  _flags_type flags;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__time_ms(
    const uint64_t & _arg)
  {
    this->time_ms = _arg;
    return *this;
  }
  Type & set__version(
    const uint8_t & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__reserved0(
    const uint8_t & _arg)
  {
    this->reserved0 = _arg;
    return *this;
  }
  Type & set__ref_station_id(
    const uint16_t & _arg)
  {
    this->ref_station_id = _arg;
    return *this;
  }
  Type & set__i_tow(
    const uint32_t & _arg)
  {
    this->i_tow = _arg;
    return *this;
  }
  Type & set__rel_pos_n(
    const int32_t & _arg)
  {
    this->rel_pos_n = _arg;
    return *this;
  }
  Type & set__rel_pos_e(
    const int32_t & _arg)
  {
    this->rel_pos_e = _arg;
    return *this;
  }
  Type & set__rel_pos_d(
    const int32_t & _arg)
  {
    this->rel_pos_d = _arg;
    return *this;
  }
  Type & set__rel_pos_length(
    const int32_t & _arg)
  {
    this->rel_pos_length = _arg;
    return *this;
  }
  Type & set__rel_pos_heading(
    const int32_t & _arg)
  {
    this->rel_pos_heading = _arg;
    return *this;
  }
  Type & set__reserved1_0(
    const uint8_t & _arg)
  {
    this->reserved1_0 = _arg;
    return *this;
  }
  Type & set__reserved1_1(
    const uint8_t & _arg)
  {
    this->reserved1_1 = _arg;
    return *this;
  }
  Type & set__reserved1_2(
    const uint8_t & _arg)
  {
    this->reserved1_2 = _arg;
    return *this;
  }
  Type & set__reserved1_3(
    const uint8_t & _arg)
  {
    this->reserved1_3 = _arg;
    return *this;
  }
  Type & set__rel_pos_hpn(
    const int8_t & _arg)
  {
    this->rel_pos_hpn = _arg;
    return *this;
  }
  Type & set__rel_pos_hpe(
    const int8_t & _arg)
  {
    this->rel_pos_hpe = _arg;
    return *this;
  }
  Type & set__rel_pos_hpd(
    const int8_t & _arg)
  {
    this->rel_pos_hpd = _arg;
    return *this;
  }
  Type & set__rel_pos_hp_length(
    const int8_t & _arg)
  {
    this->rel_pos_hp_length = _arg;
    return *this;
  }
  Type & set__acc_n(
    const uint32_t & _arg)
  {
    this->acc_n = _arg;
    return *this;
  }
  Type & set__acc_e(
    const uint32_t & _arg)
  {
    this->acc_e = _arg;
    return *this;
  }
  Type & set__acc_d(
    const uint32_t & _arg)
  {
    this->acc_d = _arg;
    return *this;
  }
  Type & set__acc_length(
    const uint32_t & _arg)
  {
    this->acc_length = _arg;
    return *this;
  }
  Type & set__acc_heading(
    const uint32_t & _arg)
  {
    this->acc_heading = _arg;
    return *this;
  }
  Type & set__reserved2_0(
    const uint8_t & _arg)
  {
    this->reserved2_0 = _arg;
    return *this;
  }
  Type & set__reserved2_1(
    const uint8_t & _arg)
  {
    this->reserved2_1 = _arg;
    return *this;
  }
  Type & set__reserved2_2(
    const uint8_t & _arg)
  {
    this->reserved2_2 = _arg;
    return *this;
  }
  Type & set__reserved2_3(
    const uint8_t & _arg)
  {
    this->reserved2_3 = _arg;
    return *this;
  }
  Type & set__flags(
    const uint32_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw
    std::shared_ptr<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw
    std::shared_ptr<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerGnssRtkRelPosNEDRaw_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time_ms != other.time_ms) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->reserved0 != other.reserved0) {
      return false;
    }
    if (this->ref_station_id != other.ref_station_id) {
      return false;
    }
    if (this->i_tow != other.i_tow) {
      return false;
    }
    if (this->rel_pos_n != other.rel_pos_n) {
      return false;
    }
    if (this->rel_pos_e != other.rel_pos_e) {
      return false;
    }
    if (this->rel_pos_d != other.rel_pos_d) {
      return false;
    }
    if (this->rel_pos_length != other.rel_pos_length) {
      return false;
    }
    if (this->rel_pos_heading != other.rel_pos_heading) {
      return false;
    }
    if (this->reserved1_0 != other.reserved1_0) {
      return false;
    }
    if (this->reserved1_1 != other.reserved1_1) {
      return false;
    }
    if (this->reserved1_2 != other.reserved1_2) {
      return false;
    }
    if (this->reserved1_3 != other.reserved1_3) {
      return false;
    }
    if (this->rel_pos_hpn != other.rel_pos_hpn) {
      return false;
    }
    if (this->rel_pos_hpe != other.rel_pos_hpe) {
      return false;
    }
    if (this->rel_pos_hpd != other.rel_pos_hpd) {
      return false;
    }
    if (this->rel_pos_hp_length != other.rel_pos_hp_length) {
      return false;
    }
    if (this->acc_n != other.acc_n) {
      return false;
    }
    if (this->acc_e != other.acc_e) {
      return false;
    }
    if (this->acc_d != other.acc_d) {
      return false;
    }
    if (this->acc_length != other.acc_length) {
      return false;
    }
    if (this->acc_heading != other.acc_heading) {
      return false;
    }
    if (this->reserved2_0 != other.reserved2_0) {
      return false;
    }
    if (this->reserved2_1 != other.reserved2_1) {
      return false;
    }
    if (this->reserved2_2 != other.reserved2_2) {
      return false;
    }
    if (this->reserved2_3 != other.reserved2_3) {
      return false;
    }
    if (this->flags != other.flags) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerGnssRtkRelPosNEDRaw_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerGnssRtkRelPosNEDRaw_

// alias to use template instance with default allocator
using MowerGnssRtkRelPosNEDRaw =
  hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__STRUCT_HPP_
