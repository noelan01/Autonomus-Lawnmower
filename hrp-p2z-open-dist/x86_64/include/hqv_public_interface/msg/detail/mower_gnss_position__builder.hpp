// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POSITION__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POSITION__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssPosition_longitude
{
public:
  explicit Init_MowerGnssPosition_longitude(::hqv_public_interface::msg::MowerGnssPosition & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssPosition longitude(::hqv_public_interface::msg::MowerGnssPosition::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPosition msg_;
};

class Init_MowerGnssPosition_latitude
{
public:
  explicit Init_MowerGnssPosition_latitude(::hqv_public_interface::msg::MowerGnssPosition & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPosition_longitude latitude(::hqv_public_interface::msg::MowerGnssPosition::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_MowerGnssPosition_longitude(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPosition msg_;
};

class Init_MowerGnssPosition_header
{
public:
  Init_MowerGnssPosition_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssPosition_latitude header(::hqv_public_interface::msg::MowerGnssPosition::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssPosition_latitude(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssPosition>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssPosition_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POSITION__BUILDER_HPP_
