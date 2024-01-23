// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssFix.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_FIX__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_FIX__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_fix__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssFix_status
{
public:
  explicit Init_MowerGnssFix_status(::hqv_public_interface::msg::MowerGnssFix & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssFix status(::hqv_public_interface::msg::MowerGnssFix::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssFix msg_;
};

class Init_MowerGnssFix_header
{
public:
  Init_MowerGnssFix_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssFix_status header(::hqv_public_interface::msg::MowerGnssFix::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssFix_status(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssFix msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssFix>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssFix_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_FIX__BUILDER_HPP_
