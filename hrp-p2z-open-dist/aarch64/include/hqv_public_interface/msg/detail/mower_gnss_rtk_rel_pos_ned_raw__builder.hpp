// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelPosNEDRaw.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_rtk_rel_pos_ned_raw__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssRtkRelPosNEDRaw_flags
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_flags(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw flags(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved2_3
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved2_3(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_flags reserved2_3(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved2_3_type arg)
  {
    msg_.reserved2_3 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_flags(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved2_2
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved2_2(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved2_3 reserved2_2(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved2_2_type arg)
  {
    msg_.reserved2_2 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved2_3(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved2_1
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved2_1(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved2_2 reserved2_1(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved2_1_type arg)
  {
    msg_.reserved2_1 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved2_2(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved2_0
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved2_0(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved2_1 reserved2_0(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved2_0_type arg)
  {
    msg_.reserved2_0 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved2_1(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_acc_heading
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_acc_heading(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved2_0 acc_heading(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_acc_heading_type arg)
  {
    msg_.acc_heading = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved2_0(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_acc_length
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_acc_length(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_acc_heading acc_length(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_acc_length_type arg)
  {
    msg_.acc_length = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_acc_heading(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_acc_d
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_acc_d(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_acc_length acc_d(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_acc_d_type arg)
  {
    msg_.acc_d = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_acc_length(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_acc_e
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_acc_e(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_acc_d acc_e(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_acc_e_type arg)
  {
    msg_.acc_e = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_acc_d(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_acc_n
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_acc_n(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_acc_e acc_n(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_acc_n_type arg)
  {
    msg_.acc_n = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_acc_e(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hp_length
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hp_length(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_acc_n rel_pos_hp_length(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_hp_length_type arg)
  {
    msg_.rel_pos_hp_length = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_acc_n(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpd
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpd(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hp_length rel_pos_hpd(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_hpd_type arg)
  {
    msg_.rel_pos_hpd = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hp_length(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpe
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpe(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpd rel_pos_hpe(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_hpe_type arg)
  {
    msg_.rel_pos_hpe = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpd(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpn
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpn(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpe rel_pos_hpn(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_hpn_type arg)
  {
    msg_.rel_pos_hpn = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpe(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved1_3
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved1_3(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpn reserved1_3(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved1_3_type arg)
  {
    msg_.reserved1_3 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_hpn(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved1_2
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved1_2(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved1_3 reserved1_2(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved1_2_type arg)
  {
    msg_.reserved1_2 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved1_3(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved1_1
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved1_1(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved1_2 reserved1_1(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved1_1_type arg)
  {
    msg_.reserved1_1 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved1_2(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved1_0
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved1_0(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved1_1 reserved1_0(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved1_0_type arg)
  {
    msg_.reserved1_0 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved1_1(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_heading
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_heading(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved1_0 rel_pos_heading(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_heading_type arg)
  {
    msg_.rel_pos_heading = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved1_0(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_length
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_length(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_heading rel_pos_length(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_length_type arg)
  {
    msg_.rel_pos_length = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_heading(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_d
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_d(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_length rel_pos_d(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_d_type arg)
  {
    msg_.rel_pos_d = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_length(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_e
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_e(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_d rel_pos_e(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_e_type arg)
  {
    msg_.rel_pos_e = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_d(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_rel_pos_n
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_rel_pos_n(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_e rel_pos_n(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_rel_pos_n_type arg)
  {
    msg_.rel_pos_n = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_e(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_i_tow
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_i_tow(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_rel_pos_n i_tow(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_i_tow_type arg)
  {
    msg_.i_tow = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_rel_pos_n(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_ref_station_id
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_ref_station_id(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_i_tow ref_station_id(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_ref_station_id_type arg)
  {
    msg_.ref_station_id = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_i_tow(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_reserved0
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_reserved0(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_ref_station_id reserved0(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_reserved0_type arg)
  {
    msg_.reserved0 = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_ref_station_id(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_version
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_version(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_reserved0 version(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_reserved0(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_time_ms
{
public:
  explicit Init_MowerGnssRtkRelPosNEDRaw_time_ms(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_version time_ms(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_time_ms_type arg)
  {
    msg_.time_ms = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_version(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

class Init_MowerGnssRtkRelPosNEDRaw_header
{
public:
  Init_MowerGnssRtkRelPosNEDRaw_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssRtkRelPosNEDRaw_time_ms header(::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssRtkRelPosNEDRaw_time_ms(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssRtkRelPosNEDRaw_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__BUILDER_HPP_
