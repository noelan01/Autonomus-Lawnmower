// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerMowerState.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_mower_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerMowerState_description
{
public:
  explicit Init_MowerMowerState_description(::hqv_public_interface::msg::MowerMowerState & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerMowerState description(::hqv_public_interface::msg::MowerMowerState::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerState msg_;
};

class Init_MowerMowerState_state
{
public:
  explicit Init_MowerMowerState_state(::hqv_public_interface::msg::MowerMowerState & msg)
  : msg_(msg)
  {}
  Init_MowerMowerState_description state(::hqv_public_interface::msg::MowerMowerState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_MowerMowerState_description(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerState msg_;
};

class Init_MowerMowerState_header
{
public:
  Init_MowerMowerState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerMowerState_state header(::hqv_public_interface::msg::MowerMowerState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerMowerState_state(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerMowerState>()
{
  return hqv_public_interface::msg::builder::Init_MowerMowerState_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__BUILDER_HPP_
