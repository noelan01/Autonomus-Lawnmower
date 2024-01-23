// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/GnssRtkCollection.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/gnss_rtk_collection__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_GnssRtkCollection_head_acc
{
public:
  explicit Init_GnssRtkCollection_head_acc(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::GnssRtkCollection head_acc(::hqv_public_interface::msg::GnssRtkCollection::_head_acc_type arg)
  {
    msg_.head_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_s_acc
{
public:
  explicit Init_GnssRtkCollection_s_acc(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_head_acc s_acc(::hqv_public_interface::msg::GnssRtkCollection::_s_acc_type arg)
  {
    msg_.s_acc = std::move(arg);
    return Init_GnssRtkCollection_head_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_head_mot
{
public:
  explicit Init_GnssRtkCollection_head_mot(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_s_acc head_mot(::hqv_public_interface::msg::GnssRtkCollection::_head_mot_type arg)
  {
    msg_.head_mot = std::move(arg);
    return Init_GnssRtkCollection_s_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_g_speed
{
public:
  explicit Init_GnssRtkCollection_g_speed(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_head_mot g_speed(::hqv_public_interface::msg::GnssRtkCollection::_g_speed_type arg)
  {
    msg_.g_speed = std::move(arg);
    return Init_GnssRtkCollection_head_mot(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_vel_d
{
public:
  explicit Init_GnssRtkCollection_vel_d(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_g_speed vel_d(::hqv_public_interface::msg::GnssRtkCollection::_vel_d_type arg)
  {
    msg_.vel_d = std::move(arg);
    return Init_GnssRtkCollection_g_speed(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_vel_e
{
public:
  explicit Init_GnssRtkCollection_vel_e(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_vel_d vel_e(::hqv_public_interface::msg::GnssRtkCollection::_vel_e_type arg)
  {
    msg_.vel_e = std::move(arg);
    return Init_GnssRtkCollection_vel_d(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_vel_n
{
public:
  explicit Init_GnssRtkCollection_vel_n(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_vel_e vel_n(::hqv_public_interface::msg::GnssRtkCollection::_vel_n_type arg)
  {
    msg_.vel_n = std::move(arg);
    return Init_GnssRtkCollection_vel_e(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_v_acc
{
public:
  explicit Init_GnssRtkCollection_v_acc(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_vel_n v_acc(::hqv_public_interface::msg::GnssRtkCollection::_v_acc_type arg)
  {
    msg_.v_acc = std::move(arg);
    return Init_GnssRtkCollection_vel_n(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_h_acc
{
public:
  explicit Init_GnssRtkCollection_h_acc(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_v_acc h_acc(::hqv_public_interface::msg::GnssRtkCollection::_h_acc_type arg)
  {
    msg_.h_acc = std::move(arg);
    return Init_GnssRtkCollection_v_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_h_msl
{
public:
  explicit Init_GnssRtkCollection_h_msl(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_h_acc h_msl(::hqv_public_interface::msg::GnssRtkCollection::_h_msl_type arg)
  {
    msg_.h_msl = std::move(arg);
    return Init_GnssRtkCollection_h_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_height
{
public:
  explicit Init_GnssRtkCollection_height(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_h_msl height(::hqv_public_interface::msg::GnssRtkCollection::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_GnssRtkCollection_h_msl(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_lat
{
public:
  explicit Init_GnssRtkCollection_lat(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_height lat(::hqv_public_interface::msg::GnssRtkCollection::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_GnssRtkCollection_height(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_lon
{
public:
  explicit Init_GnssRtkCollection_lon(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_lat lon(::hqv_public_interface::msg::GnssRtkCollection::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_GnssRtkCollection_lat(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_num_sv
{
public:
  explicit Init_GnssRtkCollection_num_sv(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_lon num_sv(::hqv_public_interface::msg::GnssRtkCollection::_num_sv_type arg)
  {
    msg_.num_sv = std::move(arg);
    return Init_GnssRtkCollection_lon(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_flags2
{
public:
  explicit Init_GnssRtkCollection_flags2(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_num_sv flags2(::hqv_public_interface::msg::GnssRtkCollection::_flags2_type arg)
  {
    msg_.flags2 = std::move(arg);
    return Init_GnssRtkCollection_num_sv(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_fix_status_flags
{
public:
  explicit Init_GnssRtkCollection_fix_status_flags(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_flags2 fix_status_flags(::hqv_public_interface::msg::GnssRtkCollection::_fix_status_flags_type arg)
  {
    msg_.fix_status_flags = std::move(arg);
    return Init_GnssRtkCollection_flags2(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_fix_type
{
public:
  explicit Init_GnssRtkCollection_fix_type(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_fix_status_flags fix_type(::hqv_public_interface::msg::GnssRtkCollection::_fix_type_type arg)
  {
    msg_.fix_type = std::move(arg);
    return Init_GnssRtkCollection_fix_status_flags(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_t_acc
{
public:
  explicit Init_GnssRtkCollection_t_acc(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_fix_type t_acc(::hqv_public_interface::msg::GnssRtkCollection::_t_acc_type arg)
  {
    msg_.t_acc = std::move(arg);
    return Init_GnssRtkCollection_fix_type(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_validity_flags
{
public:
  explicit Init_GnssRtkCollection_validity_flags(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_t_acc validity_flags(::hqv_public_interface::msg::GnssRtkCollection::_validity_flags_type arg)
  {
    msg_.validity_flags = std::move(arg);
    return Init_GnssRtkCollection_t_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_sec
{
public:
  explicit Init_GnssRtkCollection_sec(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_validity_flags sec(::hqv_public_interface::msg::GnssRtkCollection::_sec_type arg)
  {
    msg_.sec = std::move(arg);
    return Init_GnssRtkCollection_validity_flags(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_min
{
public:
  explicit Init_GnssRtkCollection_min(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_sec min(::hqv_public_interface::msg::GnssRtkCollection::_min_type arg)
  {
    msg_.min = std::move(arg);
    return Init_GnssRtkCollection_sec(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_hour
{
public:
  explicit Init_GnssRtkCollection_hour(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_min hour(::hqv_public_interface::msg::GnssRtkCollection::_hour_type arg)
  {
    msg_.hour = std::move(arg);
    return Init_GnssRtkCollection_min(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_day
{
public:
  explicit Init_GnssRtkCollection_day(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_hour day(::hqv_public_interface::msg::GnssRtkCollection::_day_type arg)
  {
    msg_.day = std::move(arg);
    return Init_GnssRtkCollection_hour(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_month
{
public:
  explicit Init_GnssRtkCollection_month(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_day month(::hqv_public_interface::msg::GnssRtkCollection::_month_type arg)
  {
    msg_.month = std::move(arg);
    return Init_GnssRtkCollection_day(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_year
{
public:
  explicit Init_GnssRtkCollection_year(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_month year(::hqv_public_interface::msg::GnssRtkCollection::_year_type arg)
  {
    msg_.year = std::move(arg);
    return Init_GnssRtkCollection_month(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_flags
{
public:
  explicit Init_GnssRtkCollection_flags(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_year flags(::hqv_public_interface::msg::GnssRtkCollection::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_GnssRtkCollection_year(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_acc_d
{
public:
  explicit Init_GnssRtkCollection_acc_d(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_flags acc_d(::hqv_public_interface::msg::GnssRtkCollection::_acc_d_type arg)
  {
    msg_.acc_d = std::move(arg);
    return Init_GnssRtkCollection_flags(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_acc_e
{
public:
  explicit Init_GnssRtkCollection_acc_e(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_acc_d acc_e(::hqv_public_interface::msg::GnssRtkCollection::_acc_e_type arg)
  {
    msg_.acc_e = std::move(arg);
    return Init_GnssRtkCollection_acc_d(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_acc_n
{
public:
  explicit Init_GnssRtkCollection_acc_n(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_acc_e acc_n(::hqv_public_interface::msg::GnssRtkCollection::_acc_n_type arg)
  {
    msg_.acc_n = std::move(arg);
    return Init_GnssRtkCollection_acc_e(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_rel_pos_hpd
{
public:
  explicit Init_GnssRtkCollection_rel_pos_hpd(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_acc_n rel_pos_hpd(::hqv_public_interface::msg::GnssRtkCollection::_rel_pos_hpd_type arg)
  {
    msg_.rel_pos_hpd = std::move(arg);
    return Init_GnssRtkCollection_acc_n(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_rel_pos_hpe
{
public:
  explicit Init_GnssRtkCollection_rel_pos_hpe(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_rel_pos_hpd rel_pos_hpe(::hqv_public_interface::msg::GnssRtkCollection::_rel_pos_hpe_type arg)
  {
    msg_.rel_pos_hpe = std::move(arg);
    return Init_GnssRtkCollection_rel_pos_hpd(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_rel_pos_hpn
{
public:
  explicit Init_GnssRtkCollection_rel_pos_hpn(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_rel_pos_hpe rel_pos_hpn(::hqv_public_interface::msg::GnssRtkCollection::_rel_pos_hpn_type arg)
  {
    msg_.rel_pos_hpn = std::move(arg);
    return Init_GnssRtkCollection_rel_pos_hpe(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_rel_pos_d
{
public:
  explicit Init_GnssRtkCollection_rel_pos_d(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_rel_pos_hpn rel_pos_d(::hqv_public_interface::msg::GnssRtkCollection::_rel_pos_d_type arg)
  {
    msg_.rel_pos_d = std::move(arg);
    return Init_GnssRtkCollection_rel_pos_hpn(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_rel_pos_e
{
public:
  explicit Init_GnssRtkCollection_rel_pos_e(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_rel_pos_d rel_pos_e(::hqv_public_interface::msg::GnssRtkCollection::_rel_pos_e_type arg)
  {
    msg_.rel_pos_e = std::move(arg);
    return Init_GnssRtkCollection_rel_pos_d(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_rel_pos_n
{
public:
  explicit Init_GnssRtkCollection_rel_pos_n(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_rel_pos_e rel_pos_n(::hqv_public_interface::msg::GnssRtkCollection::_rel_pos_n_type arg)
  {
    msg_.rel_pos_n = std::move(arg);
    return Init_GnssRtkCollection_rel_pos_e(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_i_tow
{
public:
  explicit Init_GnssRtkCollection_i_tow(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_rel_pos_n i_tow(::hqv_public_interface::msg::GnssRtkCollection::_i_tow_type arg)
  {
    msg_.i_tow = std::move(arg);
    return Init_GnssRtkCollection_rel_pos_n(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_ref_station_id
{
public:
  explicit Init_GnssRtkCollection_ref_station_id(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_i_tow ref_station_id(::hqv_public_interface::msg::GnssRtkCollection::_ref_station_id_type arg)
  {
    msg_.ref_station_id = std::move(arg);
    return Init_GnssRtkCollection_i_tow(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_time_ms
{
public:
  explicit Init_GnssRtkCollection_time_ms(::hqv_public_interface::msg::GnssRtkCollection & msg)
  : msg_(msg)
  {}
  Init_GnssRtkCollection_ref_station_id time_ms(::hqv_public_interface::msg::GnssRtkCollection::_time_ms_type arg)
  {
    msg_.time_ms = std::move(arg);
    return Init_GnssRtkCollection_ref_station_id(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

class Init_GnssRtkCollection_header
{
public:
  Init_GnssRtkCollection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GnssRtkCollection_time_ms header(::hqv_public_interface::msg::GnssRtkCollection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GnssRtkCollection_time_ms(msg_);
  }

private:
  ::hqv_public_interface::msg::GnssRtkCollection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::GnssRtkCollection>()
{
  return hqv_public_interface::msg::builder::Init_GnssRtkCollection_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__BUILDER_HPP_
