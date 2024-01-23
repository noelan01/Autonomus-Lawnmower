// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/GnssRtkCollection.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__GnssRtkCollection __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__GnssRtkCollection __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GnssRtkCollection_
{
  using Type = GnssRtkCollection_<ContainerAllocator>;

  explicit GnssRtkCollection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_ms = 0ull;
      this->ref_station_id = 0;
      this->i_tow = 0ul;
      this->rel_pos_n = 0l;
      this->rel_pos_e = 0l;
      this->rel_pos_d = 0l;
      this->rel_pos_hpn = 0;
      this->rel_pos_hpe = 0;
      this->rel_pos_hpd = 0;
      this->acc_n = 0ul;
      this->acc_e = 0ul;
      this->acc_d = 0ul;
      this->flags = 0ul;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->min = 0;
      this->sec = 0;
      this->validity_flags = 0;
      this->t_acc = 0ul;
      this->fix_type = 0;
      this->fix_status_flags = 0;
      this->flags2 = 0;
      this->num_sv = 0;
      this->lon = 0l;
      this->lat = 0l;
      this->height = 0l;
      this->h_msl = 0l;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
      this->vel_n = 0l;
      this->vel_e = 0l;
      this->vel_d = 0l;
      this->g_speed = 0l;
      this->head_mot = 0l;
      this->s_acc = 0ul;
      this->head_acc = 0ul;
    }
  }

  explicit GnssRtkCollection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_ms = 0ull;
      this->ref_station_id = 0;
      this->i_tow = 0ul;
      this->rel_pos_n = 0l;
      this->rel_pos_e = 0l;
      this->rel_pos_d = 0l;
      this->rel_pos_hpn = 0;
      this->rel_pos_hpe = 0;
      this->rel_pos_hpd = 0;
      this->acc_n = 0ul;
      this->acc_e = 0ul;
      this->acc_d = 0ul;
      this->flags = 0ul;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->min = 0;
      this->sec = 0;
      this->validity_flags = 0;
      this->t_acc = 0ul;
      this->fix_type = 0;
      this->fix_status_flags = 0;
      this->flags2 = 0;
      this->num_sv = 0;
      this->lon = 0l;
      this->lat = 0l;
      this->height = 0l;
      this->h_msl = 0l;
      this->h_acc = 0ul;
      this->v_acc = 0ul;
      this->vel_n = 0l;
      this->vel_e = 0l;
      this->vel_d = 0l;
      this->g_speed = 0l;
      this->head_mot = 0l;
      this->s_acc = 0ul;
      this->head_acc = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_ms_type =
    uint64_t;
  _time_ms_type time_ms;
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
  using _rel_pos_hpn_type =
    int8_t;
  _rel_pos_hpn_type rel_pos_hpn;
  using _rel_pos_hpe_type =
    int8_t;
  _rel_pos_hpe_type rel_pos_hpe;
  using _rel_pos_hpd_type =
    int8_t;
  _rel_pos_hpd_type rel_pos_hpd;
  using _acc_n_type =
    uint32_t;
  _acc_n_type acc_n;
  using _acc_e_type =
    uint32_t;
  _acc_e_type acc_e;
  using _acc_d_type =
    uint32_t;
  _acc_d_type acc_d;
  using _flags_type =
    uint32_t;
  _flags_type flags;
  using _year_type =
    uint16_t;
  _year_type year;
  using _month_type =
    uint8_t;
  _month_type month;
  using _day_type =
    uint8_t;
  _day_type day;
  using _hour_type =
    uint8_t;
  _hour_type hour;
  using _min_type =
    uint8_t;
  _min_type min;
  using _sec_type =
    uint8_t;
  _sec_type sec;
  using _validity_flags_type =
    uint8_t;
  _validity_flags_type validity_flags;
  using _t_acc_type =
    uint32_t;
  _t_acc_type t_acc;
  using _fix_type_type =
    uint8_t;
  _fix_type_type fix_type;
  using _fix_status_flags_type =
    uint8_t;
  _fix_status_flags_type fix_status_flags;
  using _flags2_type =
    uint8_t;
  _flags2_type flags2;
  using _num_sv_type =
    uint8_t;
  _num_sv_type num_sv;
  using _lon_type =
    int32_t;
  _lon_type lon;
  using _lat_type =
    int32_t;
  _lat_type lat;
  using _height_type =
    int32_t;
  _height_type height;
  using _h_msl_type =
    int32_t;
  _h_msl_type h_msl;
  using _h_acc_type =
    uint32_t;
  _h_acc_type h_acc;
  using _v_acc_type =
    uint32_t;
  _v_acc_type v_acc;
  using _vel_n_type =
    int32_t;
  _vel_n_type vel_n;
  using _vel_e_type =
    int32_t;
  _vel_e_type vel_e;
  using _vel_d_type =
    int32_t;
  _vel_d_type vel_d;
  using _g_speed_type =
    int32_t;
  _g_speed_type g_speed;
  using _head_mot_type =
    int32_t;
  _head_mot_type head_mot;
  using _s_acc_type =
    uint32_t;
  _s_acc_type s_acc;
  using _head_acc_type =
    uint32_t;
  _head_acc_type head_acc;

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
  Type & set__flags(
    const uint32_t & _arg)
  {
    this->flags = _arg;
    return *this;
  }
  Type & set__year(
    const uint16_t & _arg)
  {
    this->year = _arg;
    return *this;
  }
  Type & set__month(
    const uint8_t & _arg)
  {
    this->month = _arg;
    return *this;
  }
  Type & set__day(
    const uint8_t & _arg)
  {
    this->day = _arg;
    return *this;
  }
  Type & set__hour(
    const uint8_t & _arg)
  {
    this->hour = _arg;
    return *this;
  }
  Type & set__min(
    const uint8_t & _arg)
  {
    this->min = _arg;
    return *this;
  }
  Type & set__sec(
    const uint8_t & _arg)
  {
    this->sec = _arg;
    return *this;
  }
  Type & set__validity_flags(
    const uint8_t & _arg)
  {
    this->validity_flags = _arg;
    return *this;
  }
  Type & set__t_acc(
    const uint32_t & _arg)
  {
    this->t_acc = _arg;
    return *this;
  }
  Type & set__fix_type(
    const uint8_t & _arg)
  {
    this->fix_type = _arg;
    return *this;
  }
  Type & set__fix_status_flags(
    const uint8_t & _arg)
  {
    this->fix_status_flags = _arg;
    return *this;
  }
  Type & set__flags2(
    const uint8_t & _arg)
  {
    this->flags2 = _arg;
    return *this;
  }
  Type & set__num_sv(
    const uint8_t & _arg)
  {
    this->num_sv = _arg;
    return *this;
  }
  Type & set__lon(
    const int32_t & _arg)
  {
    this->lon = _arg;
    return *this;
  }
  Type & set__lat(
    const int32_t & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__h_msl(
    const int32_t & _arg)
  {
    this->h_msl = _arg;
    return *this;
  }
  Type & set__h_acc(
    const uint32_t & _arg)
  {
    this->h_acc = _arg;
    return *this;
  }
  Type & set__v_acc(
    const uint32_t & _arg)
  {
    this->v_acc = _arg;
    return *this;
  }
  Type & set__vel_n(
    const int32_t & _arg)
  {
    this->vel_n = _arg;
    return *this;
  }
  Type & set__vel_e(
    const int32_t & _arg)
  {
    this->vel_e = _arg;
    return *this;
  }
  Type & set__vel_d(
    const int32_t & _arg)
  {
    this->vel_d = _arg;
    return *this;
  }
  Type & set__g_speed(
    const int32_t & _arg)
  {
    this->g_speed = _arg;
    return *this;
  }
  Type & set__head_mot(
    const int32_t & _arg)
  {
    this->head_mot = _arg;
    return *this;
  }
  Type & set__s_acc(
    const uint32_t & _arg)
  {
    this->s_acc = _arg;
    return *this;
  }
  Type & set__head_acc(
    const uint32_t & _arg)
  {
    this->head_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__GnssRtkCollection
    std::shared_ptr<hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__GnssRtkCollection
    std::shared_ptr<hqv_public_interface::msg::GnssRtkCollection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GnssRtkCollection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time_ms != other.time_ms) {
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
    if (this->rel_pos_hpn != other.rel_pos_hpn) {
      return false;
    }
    if (this->rel_pos_hpe != other.rel_pos_hpe) {
      return false;
    }
    if (this->rel_pos_hpd != other.rel_pos_hpd) {
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
    if (this->flags != other.flags) {
      return false;
    }
    if (this->year != other.year) {
      return false;
    }
    if (this->month != other.month) {
      return false;
    }
    if (this->day != other.day) {
      return false;
    }
    if (this->hour != other.hour) {
      return false;
    }
    if (this->min != other.min) {
      return false;
    }
    if (this->sec != other.sec) {
      return false;
    }
    if (this->validity_flags != other.validity_flags) {
      return false;
    }
    if (this->t_acc != other.t_acc) {
      return false;
    }
    if (this->fix_type != other.fix_type) {
      return false;
    }
    if (this->fix_status_flags != other.fix_status_flags) {
      return false;
    }
    if (this->flags2 != other.flags2) {
      return false;
    }
    if (this->num_sv != other.num_sv) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->lat != other.lat) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->h_msl != other.h_msl) {
      return false;
    }
    if (this->h_acc != other.h_acc) {
      return false;
    }
    if (this->v_acc != other.v_acc) {
      return false;
    }
    if (this->vel_n != other.vel_n) {
      return false;
    }
    if (this->vel_e != other.vel_e) {
      return false;
    }
    if (this->vel_d != other.vel_d) {
      return false;
    }
    if (this->g_speed != other.g_speed) {
      return false;
    }
    if (this->head_mot != other.head_mot) {
      return false;
    }
    if (this->s_acc != other.s_acc) {
      return false;
    }
    if (this->head_acc != other.head_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const GnssRtkCollection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GnssRtkCollection_

// alias to use template instance with default allocator
using GnssRtkCollection =
  hqv_public_interface::msg::GnssRtkCollection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__STRUCT_HPP_
