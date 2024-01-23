// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssUnixTime.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_UNIX_TIME__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_UNIX_TIME__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_unix_time__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssUnixTime_unixtime
{
public:
  explicit Init_MowerGnssUnixTime_unixtime(::hqv_public_interface::msg::MowerGnssUnixTime & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssUnixTime unixtime(::hqv_public_interface::msg::MowerGnssUnixTime::_unixtime_type arg)
  {
    msg_.unixtime = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssUnixTime msg_;
};

class Init_MowerGnssUnixTime_itow
{
public:
  explicit Init_MowerGnssUnixTime_itow(::hqv_public_interface::msg::MowerGnssUnixTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssUnixTime_unixtime itow(::hqv_public_interface::msg::MowerGnssUnixTime::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_MowerGnssUnixTime_unixtime(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssUnixTime msg_;
};

class Init_MowerGnssUnixTime_header
{
public:
  Init_MowerGnssUnixTime_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssUnixTime_itow header(::hqv_public_interface::msg::MowerGnssUnixTime::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssUnixTime_itow(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssUnixTime msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssUnixTime>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssUnixTime_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_UNIX_TIME__BUILDER_HPP_
