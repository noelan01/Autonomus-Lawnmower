// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerMowerMode.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_mower_mode__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerMowerMode_description
{
public:
  explicit Init_MowerMowerMode_description(::hqv_public_interface::msg::MowerMowerMode & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerMowerMode description(::hqv_public_interface::msg::MowerMowerMode::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerMode msg_;
};

class Init_MowerMowerMode_mode
{
public:
  explicit Init_MowerMowerMode_mode(::hqv_public_interface::msg::MowerMowerMode & msg)
  : msg_(msg)
  {}
  Init_MowerMowerMode_description mode(::hqv_public_interface::msg::MowerMowerMode::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MowerMowerMode_description(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerMode msg_;
};

class Init_MowerMowerMode_header
{
public:
  Init_MowerMowerMode_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerMowerMode_mode header(::hqv_public_interface::msg::MowerMowerMode::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerMowerMode_mode(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerMode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerMowerMode>()
{
  return hqv_public_interface::msg::builder::Init_MowerMowerMode_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__BUILDER_HPP_
