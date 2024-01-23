// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerMowerActivity.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_mower_activity__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerMowerActivity_description
{
public:
  explicit Init_MowerMowerActivity_description(::hqv_public_interface::msg::MowerMowerActivity & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerMowerActivity description(::hqv_public_interface::msg::MowerMowerActivity::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerActivity msg_;
};

class Init_MowerMowerActivity_activity
{
public:
  explicit Init_MowerMowerActivity_activity(::hqv_public_interface::msg::MowerMowerActivity & msg)
  : msg_(msg)
  {}
  Init_MowerMowerActivity_description activity(::hqv_public_interface::msg::MowerMowerActivity::_activity_type arg)
  {
    msg_.activity = std::move(arg);
    return Init_MowerMowerActivity_description(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerActivity msg_;
};

class Init_MowerMowerActivity_header
{
public:
  Init_MowerMowerActivity_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerMowerActivity_activity header(::hqv_public_interface::msg::MowerMowerActivity::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerMowerActivity_activity(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerActivity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerMowerActivity>()
{
  return hqv_public_interface::msg::builder::Init_MowerMowerActivity_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__BUILDER_HPP_
