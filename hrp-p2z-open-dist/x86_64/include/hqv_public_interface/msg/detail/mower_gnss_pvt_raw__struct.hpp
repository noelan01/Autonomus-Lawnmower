// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hqv_public_interface:msg/MowerGnssPvtRaw.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__STRUCT_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__STRUCT_HPP_

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
# define DEPRECATED__hqv_public_interface__msg__MowerGnssPvtRaw __attribute__((deprecated))
#else
# define DEPRECATED__hqv_public_interface__msg__MowerGnssPvtRaw __declspec(deprecated)
#endif

namespace hqv_public_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerGnssPvtRaw_
{
  using Type = MowerGnssPvtRaw_<ContainerAllocator>;

  explicit MowerGnssPvtRaw_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_ms = 0ull;
      this->i_tow = 0ul;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->min = 0;
      this->sec = 0;
      this->validity_flags = 0;
      this->t_acc = 0ul;
      this->nano = 0l;
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
      this->p_dop = 0;
      this->flags3 = 0;
      this->reserved0_0 = 0;
      this->reserved0_1 = 0;
      this->reserved0_2 = 0;
      this->reserved0_3 = 0;
      this->reserved0_4 = 0;
      this->head_veh = 0l;
      this->mag_dec = 0;
      this->mag_acc = 0;
    }
  }

  explicit MowerGnssPvtRaw_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time_ms = 0ull;
      this->i_tow = 0ul;
      this->year = 0;
      this->month = 0;
      this->day = 0;
      this->hour = 0;
      this->min = 0;
      this->sec = 0;
      this->validity_flags = 0;
      this->t_acc = 0ul;
      this->nano = 0l;
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
      this->p_dop = 0;
      this->flags3 = 0;
      this->reserved0_0 = 0;
      this->reserved0_1 = 0;
      this->reserved0_2 = 0;
      this->reserved0_3 = 0;
      this->reserved0_4 = 0;
      this->head_veh = 0l;
      this->mag_dec = 0;
      this->mag_acc = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_ms_type =
    uint64_t;
  _time_ms_type time_ms;
  using _i_tow_type =
    uint32_t;
  _i_tow_type i_tow;
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
  using _nano_type =
    int32_t;
  _nano_type nano;
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
  using _p_dop_type =
    uint16_t;
  _p_dop_type p_dop;
  using _flags3_type =
    uint8_t;
  _flags3_type flags3;
  using _reserved0_0_type =
    uint8_t;
  _reserved0_0_type reserved0_0;
  using _reserved0_1_type =
    uint8_t;
  _reserved0_1_type reserved0_1;
  using _reserved0_2_type =
    uint8_t;
  _reserved0_2_type reserved0_2;
  using _reserved0_3_type =
    uint8_t;
  _reserved0_3_type reserved0_3;
  using _reserved0_4_type =
    uint8_t;
  _reserved0_4_type reserved0_4;
  using _head_veh_type =
    int32_t;
  _head_veh_type head_veh;
  using _mag_dec_type =
    int16_t;
  _mag_dec_type mag_dec;
  using _mag_acc_type =
    uint16_t;
  _mag_acc_type mag_acc;

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
  Type & set__i_tow(
    const uint32_t & _arg)
  {
    this->i_tow = _arg;
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
  Type & set__nano(
    const int32_t & _arg)
  {
    this->nano = _arg;
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
  Type & set__p_dop(
    const uint16_t & _arg)
  {
    this->p_dop = _arg;
    return *this;
  }
  Type & set__flags3(
    const uint8_t & _arg)
  {
    this->flags3 = _arg;
    return *this;
  }
  Type & set__reserved0_0(
    const uint8_t & _arg)
  {
    this->reserved0_0 = _arg;
    return *this;
  }
  Type & set__reserved0_1(
    const uint8_t & _arg)
  {
    this->reserved0_1 = _arg;
    return *this;
  }
  Type & set__reserved0_2(
    const uint8_t & _arg)
  {
    this->reserved0_2 = _arg;
    return *this;
  }
  Type & set__reserved0_3(
    const uint8_t & _arg)
  {
    this->reserved0_3 = _arg;
    return *this;
  }
  Type & set__reserved0_4(
    const uint8_t & _arg)
  {
    this->reserved0_4 = _arg;
    return *this;
  }
  Type & set__head_veh(
    const int32_t & _arg)
  {
    this->head_veh = _arg;
    return *this;
  }
  Type & set__mag_dec(
    const int16_t & _arg)
  {
    this->mag_dec = _arg;
    return *this;
  }
  Type & set__mag_acc(
    const uint16_t & _arg)
  {
    this->mag_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator> *;
  using ConstRawPtr =
    const hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssPvtRaw
    std::shared_ptr<hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hqv_public_interface__msg__MowerGnssPvtRaw
    std::shared_ptr<hqv_public_interface::msg::MowerGnssPvtRaw_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerGnssPvtRaw_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time_ms != other.time_ms) {
      return false;
    }
    if (this->i_tow != other.i_tow) {
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
    if (this->nano != other.nano) {
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
    if (this->p_dop != other.p_dop) {
      return false;
    }
    if (this->flags3 != other.flags3) {
      return false;
    }
    if (this->reserved0_0 != other.reserved0_0) {
      return false;
    }
    if (this->reserved0_1 != other.reserved0_1) {
      return false;
    }
    if (this->reserved0_2 != other.reserved0_2) {
      return false;
    }
    if (this->reserved0_3 != other.reserved0_3) {
      return false;
    }
    if (this->reserved0_4 != other.reserved0_4) {
      return false;
    }
    if (this->head_veh != other.head_veh) {
      return false;
    }
    if (this->mag_dec != other.mag_dec) {
      return false;
    }
    if (this->mag_acc != other.mag_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerGnssPvtRaw_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerGnssPvtRaw_

// alias to use template instance with default allocator
using MowerGnssPvtRaw =
  hqv_public_interface::msg::MowerGnssPvtRaw_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__STRUCT_HPP_
