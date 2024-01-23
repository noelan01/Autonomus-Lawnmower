// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerBatteryVoltage.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BATTERY_VOLTAGE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BATTERY_VOLTAGE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_battery_voltage__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerBatteryVoltage_voltage
{
public:
  explicit Init_MowerBatteryVoltage_voltage(::hqv_public_interface::msg::MowerBatteryVoltage & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerBatteryVoltage voltage(::hqv_public_interface::msg::MowerBatteryVoltage::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBatteryVoltage msg_;
};

class Init_MowerBatteryVoltage_header
{
public:
  Init_MowerBatteryVoltage_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerBatteryVoltage_voltage header(::hqv_public_interface::msg::MowerBatteryVoltage::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerBatteryVoltage_voltage(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBatteryVoltage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerBatteryVoltage>()
{
  return hqv_public_interface::msg::builder::Init_MowerBatteryVoltage_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BATTERY_VOLTAGE__BUILDER_HPP_
