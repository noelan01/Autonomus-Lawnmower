// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssHdop.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HDOP__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HDOP__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_hdop__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssHdop_strength
{
public:
  explicit Init_MowerGnssHdop_strength(::hqv_public_interface::msg::MowerGnssHdop & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssHdop strength(::hqv_public_interface::msg::MowerGnssHdop::_strength_type arg)
  {
    msg_.strength = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssHdop msg_;
};

class Init_MowerGnssHdop_header
{
public:
  Init_MowerGnssHdop_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssHdop_strength header(::hqv_public_interface::msg::MowerGnssHdop::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssHdop_strength(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssHdop msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssHdop>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssHdop_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HDOP__BUILDER_HPP_
