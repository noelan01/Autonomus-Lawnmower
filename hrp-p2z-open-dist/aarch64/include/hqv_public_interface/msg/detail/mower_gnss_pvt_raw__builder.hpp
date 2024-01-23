// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssPvtRaw.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_pvt_raw__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssPvtRaw_mag_acc
{
public:
  explicit Init_MowerGnssPvtRaw_mag_acc(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssPvtRaw mag_acc(::hqv_public_interface::msg::MowerGnssPvtRaw::_mag_acc_type arg)
  {
    msg_.mag_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_mag_dec
{
public:
  explicit Init_MowerGnssPvtRaw_mag_dec(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_mag_acc mag_dec(::hqv_public_interface::msg::MowerGnssPvtRaw::_mag_dec_type arg)
  {
    msg_.mag_dec = std::move(arg);
    return Init_MowerGnssPvtRaw_mag_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_head_veh
{
public:
  explicit Init_MowerGnssPvtRaw_head_veh(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_mag_dec head_veh(::hqv_public_interface::msg::MowerGnssPvtRaw::_head_veh_type arg)
  {
    msg_.head_veh = std::move(arg);
    return Init_MowerGnssPvtRaw_mag_dec(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_reserved0_4
{
public:
  explicit Init_MowerGnssPvtRaw_reserved0_4(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_head_veh reserved0_4(::hqv_public_interface::msg::MowerGnssPvtRaw::_reserved0_4_type arg)
  {
    msg_.reserved0_4 = std::move(arg);
    return Init_MowerGnssPvtRaw_head_veh(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_reserved0_3
{
public:
  explicit Init_MowerGnssPvtRaw_reserved0_3(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_reserved0_4 reserved0_3(::hqv_public_interface::msg::MowerGnssPvtRaw::_reserved0_3_type arg)
  {
    msg_.reserved0_3 = std::move(arg);
    return Init_MowerGnssPvtRaw_reserved0_4(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_reserved0_2
{
public:
  explicit Init_MowerGnssPvtRaw_reserved0_2(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_reserved0_3 reserved0_2(::hqv_public_interface::msg::MowerGnssPvtRaw::_reserved0_2_type arg)
  {
    msg_.reserved0_2 = std::move(arg);
    return Init_MowerGnssPvtRaw_reserved0_3(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_reserved0_1
{
public:
  explicit Init_MowerGnssPvtRaw_reserved0_1(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_reserved0_2 reserved0_1(::hqv_public_interface::msg::MowerGnssPvtRaw::_reserved0_1_type arg)
  {
    msg_.reserved0_1 = std::move(arg);
    return Init_MowerGnssPvtRaw_reserved0_2(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_reserved0_0
{
public:
  explicit Init_MowerGnssPvtRaw_reserved0_0(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_reserved0_1 reserved0_0(::hqv_public_interface::msg::MowerGnssPvtRaw::_reserved0_0_type arg)
  {
    msg_.reserved0_0 = std::move(arg);
    return Init_MowerGnssPvtRaw_reserved0_1(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_flags3
{
public:
  explicit Init_MowerGnssPvtRaw_flags3(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_reserved0_0 flags3(::hqv_public_interface::msg::MowerGnssPvtRaw::_flags3_type arg)
  {
    msg_.flags3 = std::move(arg);
    return Init_MowerGnssPvtRaw_reserved0_0(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_p_dop
{
public:
  explicit Init_MowerGnssPvtRaw_p_dop(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_flags3 p_dop(::hqv_public_interface::msg::MowerGnssPvtRaw::_p_dop_type arg)
  {
    msg_.p_dop = std::move(arg);
    return Init_MowerGnssPvtRaw_flags3(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_head_acc
{
public:
  explicit Init_MowerGnssPvtRaw_head_acc(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_p_dop head_acc(::hqv_public_interface::msg::MowerGnssPvtRaw::_head_acc_type arg)
  {
    msg_.head_acc = std::move(arg);
    return Init_MowerGnssPvtRaw_p_dop(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_s_acc
{
public:
  explicit Init_MowerGnssPvtRaw_s_acc(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_head_acc s_acc(::hqv_public_interface::msg::MowerGnssPvtRaw::_s_acc_type arg)
  {
    msg_.s_acc = std::move(arg);
    return Init_MowerGnssPvtRaw_head_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_head_mot
{
public:
  explicit Init_MowerGnssPvtRaw_head_mot(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_s_acc head_mot(::hqv_public_interface::msg::MowerGnssPvtRaw::_head_mot_type arg)
  {
    msg_.head_mot = std::move(arg);
    return Init_MowerGnssPvtRaw_s_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_g_speed
{
public:
  explicit Init_MowerGnssPvtRaw_g_speed(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_head_mot g_speed(::hqv_public_interface::msg::MowerGnssPvtRaw::_g_speed_type arg)
  {
    msg_.g_speed = std::move(arg);
    return Init_MowerGnssPvtRaw_head_mot(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_vel_d
{
public:
  explicit Init_MowerGnssPvtRaw_vel_d(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_g_speed vel_d(::hqv_public_interface::msg::MowerGnssPvtRaw::_vel_d_type arg)
  {
    msg_.vel_d = std::move(arg);
    return Init_MowerGnssPvtRaw_g_speed(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_vel_e
{
public:
  explicit Init_MowerGnssPvtRaw_vel_e(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_vel_d vel_e(::hqv_public_interface::msg::MowerGnssPvtRaw::_vel_e_type arg)
  {
    msg_.vel_e = std::move(arg);
    return Init_MowerGnssPvtRaw_vel_d(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_vel_n
{
public:
  explicit Init_MowerGnssPvtRaw_vel_n(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_vel_e vel_n(::hqv_public_interface::msg::MowerGnssPvtRaw::_vel_n_type arg)
  {
    msg_.vel_n = std::move(arg);
    return Init_MowerGnssPvtRaw_vel_e(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_v_acc
{
public:
  explicit Init_MowerGnssPvtRaw_v_acc(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_vel_n v_acc(::hqv_public_interface::msg::MowerGnssPvtRaw::_v_acc_type arg)
  {
    msg_.v_acc = std::move(arg);
    return Init_MowerGnssPvtRaw_vel_n(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_h_acc
{
public:
  explicit Init_MowerGnssPvtRaw_h_acc(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_v_acc h_acc(::hqv_public_interface::msg::MowerGnssPvtRaw::_h_acc_type arg)
  {
    msg_.h_acc = std::move(arg);
    return Init_MowerGnssPvtRaw_v_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_h_msl
{
public:
  explicit Init_MowerGnssPvtRaw_h_msl(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_h_acc h_msl(::hqv_public_interface::msg::MowerGnssPvtRaw::_h_msl_type arg)
  {
    msg_.h_msl = std::move(arg);
    return Init_MowerGnssPvtRaw_h_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_height
{
public:
  explicit Init_MowerGnssPvtRaw_height(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_h_msl height(::hqv_public_interface::msg::MowerGnssPvtRaw::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_MowerGnssPvtRaw_h_msl(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_lat
{
public:
  explicit Init_MowerGnssPvtRaw_lat(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_height lat(::hqv_public_interface::msg::MowerGnssPvtRaw::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_MowerGnssPvtRaw_height(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_lon
{
public:
  explicit Init_MowerGnssPvtRaw_lon(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_lat lon(::hqv_public_interface::msg::MowerGnssPvtRaw::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_MowerGnssPvtRaw_lat(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_num_sv
{
public:
  explicit Init_MowerGnssPvtRaw_num_sv(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_lon num_sv(::hqv_public_interface::msg::MowerGnssPvtRaw::_num_sv_type arg)
  {
    msg_.num_sv = std::move(arg);
    return Init_MowerGnssPvtRaw_lon(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_flags2
{
public:
  explicit Init_MowerGnssPvtRaw_flags2(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_num_sv flags2(::hqv_public_interface::msg::MowerGnssPvtRaw::_flags2_type arg)
  {
    msg_.flags2 = std::move(arg);
    return Init_MowerGnssPvtRaw_num_sv(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_fix_status_flags
{
public:
  explicit Init_MowerGnssPvtRaw_fix_status_flags(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_flags2 fix_status_flags(::hqv_public_interface::msg::MowerGnssPvtRaw::_fix_status_flags_type arg)
  {
    msg_.fix_status_flags = std::move(arg);
    return Init_MowerGnssPvtRaw_flags2(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_fix_type
{
public:
  explicit Init_MowerGnssPvtRaw_fix_type(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_fix_status_flags fix_type(::hqv_public_interface::msg::MowerGnssPvtRaw::_fix_type_type arg)
  {
    msg_.fix_type = std::move(arg);
    return Init_MowerGnssPvtRaw_fix_status_flags(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_nano
{
public:
  explicit Init_MowerGnssPvtRaw_nano(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_fix_type nano(::hqv_public_interface::msg::MowerGnssPvtRaw::_nano_type arg)
  {
    msg_.nano = std::move(arg);
    return Init_MowerGnssPvtRaw_fix_type(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_t_acc
{
public:
  explicit Init_MowerGnssPvtRaw_t_acc(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_nano t_acc(::hqv_public_interface::msg::MowerGnssPvtRaw::_t_acc_type arg)
  {
    msg_.t_acc = std::move(arg);
    return Init_MowerGnssPvtRaw_nano(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_validity_flags
{
public:
  explicit Init_MowerGnssPvtRaw_validity_flags(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_t_acc validity_flags(::hqv_public_interface::msg::MowerGnssPvtRaw::_validity_flags_type arg)
  {
    msg_.validity_flags = std::move(arg);
    return Init_MowerGnssPvtRaw_t_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_sec
{
public:
  explicit Init_MowerGnssPvtRaw_sec(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_validity_flags sec(::hqv_public_interface::msg::MowerGnssPvtRaw::_sec_type arg)
  {
    msg_.sec = std::move(arg);
    return Init_MowerGnssPvtRaw_validity_flags(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_min
{
public:
  explicit Init_MowerGnssPvtRaw_min(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_sec min(::hqv_public_interface::msg::MowerGnssPvtRaw::_min_type arg)
  {
    msg_.min = std::move(arg);
    return Init_MowerGnssPvtRaw_sec(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_hour
{
public:
  explicit Init_MowerGnssPvtRaw_hour(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_min hour(::hqv_public_interface::msg::MowerGnssPvtRaw::_hour_type arg)
  {
    msg_.hour = std::move(arg);
    return Init_MowerGnssPvtRaw_min(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_day
{
public:
  explicit Init_MowerGnssPvtRaw_day(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_hour day(::hqv_public_interface::msg::MowerGnssPvtRaw::_day_type arg)
  {
    msg_.day = std::move(arg);
    return Init_MowerGnssPvtRaw_hour(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_month
{
public:
  explicit Init_MowerGnssPvtRaw_month(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_day month(::hqv_public_interface::msg::MowerGnssPvtRaw::_month_type arg)
  {
    msg_.month = std::move(arg);
    return Init_MowerGnssPvtRaw_day(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_year
{
public:
  explicit Init_MowerGnssPvtRaw_year(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_month year(::hqv_public_interface::msg::MowerGnssPvtRaw::_year_type arg)
  {
    msg_.year = std::move(arg);
    return Init_MowerGnssPvtRaw_month(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_i_tow
{
public:
  explicit Init_MowerGnssPvtRaw_i_tow(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_year i_tow(::hqv_public_interface::msg::MowerGnssPvtRaw::_i_tow_type arg)
  {
    msg_.i_tow = std::move(arg);
    return Init_MowerGnssPvtRaw_year(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_time_ms
{
public:
  explicit Init_MowerGnssPvtRaw_time_ms(::hqv_public_interface::msg::MowerGnssPvtRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPvtRaw_i_tow time_ms(::hqv_public_interface::msg::MowerGnssPvtRaw::_time_ms_type arg)
  {
    msg_.time_ms = std::move(arg);
    return Init_MowerGnssPvtRaw_i_tow(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

class Init_MowerGnssPvtRaw_header
{
public:
  Init_MowerGnssPvtRaw_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssPvtRaw_time_ms header(::hqv_public_interface::msg::MowerGnssPvtRaw::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssPvtRaw_time_ms(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPvtRaw msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssPvtRaw>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssPvtRaw_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__BUILDER_HPP_
