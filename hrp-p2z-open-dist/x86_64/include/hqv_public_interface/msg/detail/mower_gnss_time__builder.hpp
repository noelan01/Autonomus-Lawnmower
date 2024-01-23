// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssTime.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_time__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssTime_datevalid
{
public:
  explicit Init_MowerGnssTime_datevalid(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssTime datevalid(::hqv_public_interface::msg::MowerGnssTime::_datevalid_type arg)
  {
    msg_.datevalid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_timevalid
{
public:
  explicit Init_MowerGnssTime_timevalid(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_datevalid timevalid(::hqv_public_interface::msg::MowerGnssTime::_timevalid_type arg)
  {
    msg_.timevalid = std::move(arg);
    return Init_MowerGnssTime_datevalid(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_timeaccuracy
{
public:
  explicit Init_MowerGnssTime_timeaccuracy(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_timevalid timeaccuracy(::hqv_public_interface::msg::MowerGnssTime::_timeaccuracy_type arg)
  {
    msg_.timeaccuracy = std::move(arg);
    return Init_MowerGnssTime_timevalid(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_nano
{
public:
  explicit Init_MowerGnssTime_nano(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_timeaccuracy nano(::hqv_public_interface::msg::MowerGnssTime::_nano_type arg)
  {
    msg_.nano = std::move(arg);
    return Init_MowerGnssTime_timeaccuracy(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_second
{
public:
  explicit Init_MowerGnssTime_second(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_nano second(::hqv_public_interface::msg::MowerGnssTime::_second_type arg)
  {
    msg_.second = std::move(arg);
    return Init_MowerGnssTime_nano(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_minute
{
public:
  explicit Init_MowerGnssTime_minute(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_second minute(::hqv_public_interface::msg::MowerGnssTime::_minute_type arg)
  {
    msg_.minute = std::move(arg);
    return Init_MowerGnssTime_second(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_hour
{
public:
  explicit Init_MowerGnssTime_hour(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_minute hour(::hqv_public_interface::msg::MowerGnssTime::_hour_type arg)
  {
    msg_.hour = std::move(arg);
    return Init_MowerGnssTime_minute(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_day
{
public:
  explicit Init_MowerGnssTime_day(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_hour day(::hqv_public_interface::msg::MowerGnssTime::_day_type arg)
  {
    msg_.day = std::move(arg);
    return Init_MowerGnssTime_hour(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_month
{
public:
  explicit Init_MowerGnssTime_month(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_day month(::hqv_public_interface::msg::MowerGnssTime::_month_type arg)
  {
    msg_.month = std::move(arg);
    return Init_MowerGnssTime_day(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_year
{
public:
  explicit Init_MowerGnssTime_year(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_month year(::hqv_public_interface::msg::MowerGnssTime::_year_type arg)
  {
    msg_.year = std::move(arg);
    return Init_MowerGnssTime_month(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_itow
{
public:
  explicit Init_MowerGnssTime_itow(::hqv_public_interface::msg::MowerGnssTime & msg)
  : msg_(msg)
  {}
  Init_MowerGnssTime_year itow(::hqv_public_interface::msg::MowerGnssTime::_itow_type arg)
  {
    msg_.itow = std::move(arg);
    return Init_MowerGnssTime_year(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

class Init_MowerGnssTime_header
{
public:
  Init_MowerGnssTime_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssTime_itow header(::hqv_public_interface::msg::MowerGnssTime::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssTime_itow(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssTime msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssTime>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssTime_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__BUILDER_HPP_
