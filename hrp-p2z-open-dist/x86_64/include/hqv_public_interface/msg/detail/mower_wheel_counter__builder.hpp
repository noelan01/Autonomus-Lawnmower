// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerWheelCounter.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_WHEEL_COUNTER__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_WHEEL_COUNTER__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_wheel_counter__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerWheelCounter_deltadistance
{
public:
  explicit Init_MowerWheelCounter_deltadistance(::hqv_public_interface::msg::MowerWheelCounter & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerWheelCounter deltadistance(::hqv_public_interface::msg::MowerWheelCounter::_deltadistance_type arg)
  {
    msg_.deltadistance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelCounter msg_;
};

class Init_MowerWheelCounter_free
{
public:
  explicit Init_MowerWheelCounter_free(::hqv_public_interface::msg::MowerWheelCounter & msg)
  : msg_(msg)
  {}
  Init_MowerWheelCounter_deltadistance free(::hqv_public_interface::msg::MowerWheelCounter::_free_type arg)
  {
    msg_.free = std::move(arg);
    return Init_MowerWheelCounter_deltadistance(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelCounter msg_;
};

class Init_MowerWheelCounter_counter
{
public:
  explicit Init_MowerWheelCounter_counter(::hqv_public_interface::msg::MowerWheelCounter & msg)
  : msg_(msg)
  {}
  Init_MowerWheelCounter_free counter(::hqv_public_interface::msg::MowerWheelCounter::_counter_type arg)
  {
    msg_.counter = std::move(arg);
    return Init_MowerWheelCounter_free(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelCounter msg_;
};

class Init_MowerWheelCounter_power
{
public:
  explicit Init_MowerWheelCounter_power(::hqv_public_interface::msg::MowerWheelCounter & msg)
  : msg_(msg)
  {}
  Init_MowerWheelCounter_counter power(::hqv_public_interface::msg::MowerWheelCounter::_power_type arg)
  {
    msg_.power = std::move(arg);
    return Init_MowerWheelCounter_counter(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelCounter msg_;
};

class Init_MowerWheelCounter_index
{
public:
  explicit Init_MowerWheelCounter_index(::hqv_public_interface::msg::MowerWheelCounter & msg)
  : msg_(msg)
  {}
  Init_MowerWheelCounter_power index(::hqv_public_interface::msg::MowerWheelCounter::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_MowerWheelCounter_power(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelCounter msg_;
};

class Init_MowerWheelCounter_header
{
public:
  Init_MowerWheelCounter_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerWheelCounter_index header(::hqv_public_interface::msg::MowerWheelCounter::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerWheelCounter_index(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerWheelCounter msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerWheelCounter>()
{
  return hqv_public_interface::msg::builder::Init_MowerWheelCounter_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_WHEEL_COUNTER__BUILDER_HPP_
