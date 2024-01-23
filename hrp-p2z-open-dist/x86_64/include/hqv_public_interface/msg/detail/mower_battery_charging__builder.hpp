// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerBatteryCharging.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BATTERY_CHARGING__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BATTERY_CHARGING__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_battery_charging__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerBatteryCharging_charging
{
public:
  explicit Init_MowerBatteryCharging_charging(::hqv_public_interface::msg::MowerBatteryCharging & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerBatteryCharging charging(::hqv_public_interface::msg::MowerBatteryCharging::_charging_type arg)
  {
    msg_.charging = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBatteryCharging msg_;
};

class Init_MowerBatteryCharging_header
{
public:
  Init_MowerBatteryCharging_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerBatteryCharging_charging header(::hqv_public_interface::msg::MowerBatteryCharging::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerBatteryCharging_charging(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBatteryCharging msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerBatteryCharging>()
{
  return hqv_public_interface::msg::builder::Init_MowerBatteryCharging_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BATTERY_CHARGING__BUILDER_HPP_
