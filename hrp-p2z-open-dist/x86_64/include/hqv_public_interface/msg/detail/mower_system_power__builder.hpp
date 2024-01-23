// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerSystemPower.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_system_power__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerSystemPower_countdown
{
public:
  explicit Init_MowerSystemPower_countdown(::hqv_public_interface::msg::MowerSystemPower & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerSystemPower countdown(::hqv_public_interface::msg::MowerSystemPower::_countdown_type arg)
  {
    msg_.countdown = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSystemPower msg_;
};

class Init_MowerSystemPower_mode
{
public:
  explicit Init_MowerSystemPower_mode(::hqv_public_interface::msg::MowerSystemPower & msg)
  : msg_(msg)
  {}
  Init_MowerSystemPower_countdown mode(::hqv_public_interface::msg::MowerSystemPower::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MowerSystemPower_countdown(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSystemPower msg_;
};

class Init_MowerSystemPower_header
{
public:
  Init_MowerSystemPower_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerSystemPower_mode header(::hqv_public_interface::msg::MowerSystemPower::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerSystemPower_mode(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSystemPower msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerSystemPower>()
{
  return hqv_public_interface::msg::builder::Init_MowerSystemPower_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__BUILDER_HPP_
