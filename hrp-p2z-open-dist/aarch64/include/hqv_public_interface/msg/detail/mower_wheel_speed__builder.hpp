// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerWheelSpeed.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_WHEEL_SPEED__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_WHEEL_SPEED__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_wheel_speed__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerWheelSpeed_isrunning
{
public:
  explicit Init_MowerWheelSpeed_isrunning(::hqv_public_interface::msg::MowerWheelSpeed & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerWheelSpeed isrunning(::hqv_public_interface::msg::MowerWheelSpeed::_isrunning_type arg)
  {
    msg_.isrunning = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelSpeed msg_;
};

class Init_MowerWheelSpeed_speed
{
public:
  explicit Init_MowerWheelSpeed_speed(::hqv_public_interface::msg::MowerWheelSpeed & msg)
  : msg_(msg)
  {}
  Init_MowerWheelSpeed_isrunning speed(::hqv_public_interface::msg::MowerWheelSpeed::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_MowerWheelSpeed_isrunning(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelSpeed msg_;
};

class Init_MowerWheelSpeed_current
{
public:
  explicit Init_MowerWheelSpeed_current(::hqv_public_interface::msg::MowerWheelSpeed & msg)
  : msg_(msg)
  {}
  Init_MowerWheelSpeed_speed current(::hqv_public_interface::msg::MowerWheelSpeed::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_MowerWheelSpeed_speed(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelSpeed msg_;
};

class Init_MowerWheelSpeed_state
{
public:
  explicit Init_MowerWheelSpeed_state(::hqv_public_interface::msg::MowerWheelSpeed & msg)
  : msg_(msg)
  {}
  Init_MowerWheelSpeed_current state(::hqv_public_interface::msg::MowerWheelSpeed::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_MowerWheelSpeed_current(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelSpeed msg_;
};

class Init_MowerWheelSpeed_index
{
public:
  explicit Init_MowerWheelSpeed_index(::hqv_public_interface::msg::MowerWheelSpeed & msg)
  : msg_(msg)
  {}
  Init_MowerWheelSpeed_state index(::hqv_public_interface::msg::MowerWheelSpeed::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_MowerWheelSpeed_state(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelSpeed msg_;
};

class Init_MowerWheelSpeed_header
{
public:
  Init_MowerWheelSpeed_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerWheelSpeed_index header(::hqv_public_interface::msg::MowerWheelSpeed::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerWheelSpeed_index(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelSpeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerWheelSpeed>()
{
  return hqv_public_interface::msg::builder::Init_MowerWheelSpeed_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_WHEEL_SPEED__BUILDER_HPP_
