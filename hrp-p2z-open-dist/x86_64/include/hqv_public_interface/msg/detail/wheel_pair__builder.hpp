// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/WheelPair.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/wheel_pair__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_WheelPair_isrunning_right
{
public:
  explicit Init_WheelPair_isrunning_right(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::WheelPair isrunning_right(::hqv_public_interface::msg::WheelPair::_isrunning_right_type arg)
  {
    msg_.isrunning_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_isrunning_left
{
public:
  explicit Init_WheelPair_isrunning_left(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_isrunning_right isrunning_left(::hqv_public_interface::msg::WheelPair::_isrunning_left_type arg)
  {
    msg_.isrunning_left = std::move(arg);
    return Init_WheelPair_isrunning_right(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_speed_right
{
public:
  explicit Init_WheelPair_speed_right(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_isrunning_left speed_right(::hqv_public_interface::msg::WheelPair::_speed_right_type arg)
  {
    msg_.speed_right = std::move(arg);
    return Init_WheelPair_isrunning_left(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_speed_left
{
public:
  explicit Init_WheelPair_speed_left(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_speed_right speed_left(::hqv_public_interface::msg::WheelPair::_speed_left_type arg)
  {
    msg_.speed_left = std::move(arg);
    return Init_WheelPair_speed_right(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_current_right
{
public:
  explicit Init_WheelPair_current_right(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_speed_left current_right(::hqv_public_interface::msg::WheelPair::_current_right_type arg)
  {
    msg_.current_right = std::move(arg);
    return Init_WheelPair_speed_left(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_current_left
{
public:
  explicit Init_WheelPair_current_left(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_current_right current_left(::hqv_public_interface::msg::WheelPair::_current_left_type arg)
  {
    msg_.current_left = std::move(arg);
    return Init_WheelPair_current_right(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_state_right
{
public:
  explicit Init_WheelPair_state_right(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_current_left state_right(::hqv_public_interface::msg::WheelPair::_state_right_type arg)
  {
    msg_.state_right = std::move(arg);
    return Init_WheelPair_current_left(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_state_left
{
public:
  explicit Init_WheelPair_state_left(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_state_right state_left(::hqv_public_interface::msg::WheelPair::_state_left_type arg)
  {
    msg_.state_left = std::move(arg);
    return Init_WheelPair_state_right(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_deltadistance_right
{
public:
  explicit Init_WheelPair_deltadistance_right(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_state_left deltadistance_right(::hqv_public_interface::msg::WheelPair::_deltadistance_right_type arg)
  {
    msg_.deltadistance_right = std::move(arg);
    return Init_WheelPair_state_left(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_deltadistance_left
{
public:
  explicit Init_WheelPair_deltadistance_left(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_deltadistance_right deltadistance_left(::hqv_public_interface::msg::WheelPair::_deltadistance_left_type arg)
  {
    msg_.deltadistance_left = std::move(arg);
    return Init_WheelPair_deltadistance_right(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_free_right
{
public:
  explicit Init_WheelPair_free_right(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_deltadistance_left free_right(::hqv_public_interface::msg::WheelPair::_free_right_type arg)
  {
    msg_.free_right = std::move(arg);
    return Init_WheelPair_deltadistance_left(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_free_left
{
public:
  explicit Init_WheelPair_free_left(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_free_right free_left(::hqv_public_interface::msg::WheelPair::_free_left_type arg)
  {
    msg_.free_left = std::move(arg);
    return Init_WheelPair_free_right(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_counter_right
{
public:
  explicit Init_WheelPair_counter_right(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_free_left counter_right(::hqv_public_interface::msg::WheelPair::_counter_right_type arg)
  {
    msg_.counter_right = std::move(arg);
    return Init_WheelPair_free_left(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_counter_left
{
public:
  explicit Init_WheelPair_counter_left(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_counter_right counter_left(::hqv_public_interface::msg::WheelPair::_counter_left_type arg)
  {
    msg_.counter_left = std::move(arg);
    return Init_WheelPair_counter_right(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_power_right
{
public:
  explicit Init_WheelPair_power_right(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_counter_left power_right(::hqv_public_interface::msg::WheelPair::_power_right_type arg)
  {
    msg_.power_right = std::move(arg);
    return Init_WheelPair_counter_left(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_power_left
{
public:
  explicit Init_WheelPair_power_left(::hqv_public_interface::msg::WheelPair & msg)
  : msg_(msg)
  {}
  Init_WheelPair_power_right power_left(::hqv_public_interface::msg::WheelPair::_power_left_type arg)
  {
    msg_.power_left = std::move(arg);
    return Init_WheelPair_power_right(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

class Init_WheelPair_header
{
public:
  Init_WheelPair_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WheelPair_power_left header(::hqv_public_interface::msg::WheelPair::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WheelPair_power_left(msg_);
  }

private:
  ::hqv_public_interface::msg::WheelPair msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::WheelPair>()
{
  return hqv_public_interface::msg::builder::Init_WheelPair_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__BUILDER_HPP_
