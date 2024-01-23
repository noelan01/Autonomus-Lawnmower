// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerBladeMotorDataUpdate.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_blade_motor_data_update__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerBladeMotorDataUpdate_current
{
public:
  explicit Init_MowerBladeMotorDataUpdate_current(::hqv_public_interface::msg::MowerBladeMotorDataUpdate & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerBladeMotorDataUpdate current(::hqv_public_interface::msg::MowerBladeMotorDataUpdate::_current_type arg)
  {
    msg_.current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBladeMotorDataUpdate msg_;
};

class Init_MowerBladeMotorDataUpdate_rpm
{
public:
  explicit Init_MowerBladeMotorDataUpdate_rpm(::hqv_public_interface::msg::MowerBladeMotorDataUpdate & msg)
  : msg_(msg)
  {}
  Init_MowerBladeMotorDataUpdate_current rpm(::hqv_public_interface::msg::MowerBladeMotorDataUpdate::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_MowerBladeMotorDataUpdate_current(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBladeMotorDataUpdate msg_;
};

class Init_MowerBladeMotorDataUpdate_direction
{
public:
  explicit Init_MowerBladeMotorDataUpdate_direction(::hqv_public_interface::msg::MowerBladeMotorDataUpdate & msg)
  : msg_(msg)
  {}
  Init_MowerBladeMotorDataUpdate_rpm direction(::hqv_public_interface::msg::MowerBladeMotorDataUpdate::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_MowerBladeMotorDataUpdate_rpm(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBladeMotorDataUpdate msg_;
};

class Init_MowerBladeMotorDataUpdate_state
{
public:
  explicit Init_MowerBladeMotorDataUpdate_state(::hqv_public_interface::msg::MowerBladeMotorDataUpdate & msg)
  : msg_(msg)
  {}
  Init_MowerBladeMotorDataUpdate_direction state(::hqv_public_interface::msg::MowerBladeMotorDataUpdate::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_MowerBladeMotorDataUpdate_direction(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBladeMotorDataUpdate msg_;
};

class Init_MowerBladeMotorDataUpdate_header
{
public:
  Init_MowerBladeMotorDataUpdate_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerBladeMotorDataUpdate_state header(::hqv_public_interface::msg::MowerBladeMotorDataUpdate::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerBladeMotorDataUpdate_state(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerBladeMotorDataUpdate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerBladeMotorDataUpdate>()
{
  return hqv_public_interface::msg::builder::Init_MowerBladeMotorDataUpdate_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__BUILDER_HPP_
