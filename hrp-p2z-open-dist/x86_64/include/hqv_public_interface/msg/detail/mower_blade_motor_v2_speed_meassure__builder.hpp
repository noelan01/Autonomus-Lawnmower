// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerBladeMotorV2SpeedMeassure.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_SPEED_MEASSURE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_SPEED_MEASSURE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_blade_motor_v2_speed_meassure__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerBladeMotorV2SpeedMeassure_speed
{
public:
  explicit Init_MowerBladeMotorV2SpeedMeassure_speed(::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure speed(::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure msg_;
};

class Init_MowerBladeMotorV2SpeedMeassure_index
{
public:
  explicit Init_MowerBladeMotorV2SpeedMeassure_index(::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure & msg)
  : msg_(msg)
  {}
  Init_MowerBladeMotorV2SpeedMeassure_speed index(::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_MowerBladeMotorV2SpeedMeassure_speed(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure msg_;
};

class Init_MowerBladeMotorV2SpeedMeassure_header
{
public:
  Init_MowerBladeMotorV2SpeedMeassure_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerBladeMotorV2SpeedMeassure_index header(::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerBladeMotorV2SpeedMeassure_index(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerBladeMotorV2SpeedMeassure>()
{
  return hqv_public_interface::msg::builder::Init_MowerBladeMotorV2SpeedMeassure_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_V2_SPEED_MEASSURE__BUILDER_HPP_
