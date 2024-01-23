// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/TractionDriveCommand.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__TRACTION_DRIVE_COMMAND__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__TRACTION_DRIVE_COMMAND__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/traction_drive_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_TractionDriveCommand_steering
{
public:
  explicit Init_TractionDriveCommand_steering(::hqv_public_interface::msg::TractionDriveCommand & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::TractionDriveCommand steering(::hqv_public_interface::msg::TractionDriveCommand::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::TractionDriveCommand msg_;
};

class Init_TractionDriveCommand_speed
{
public:
  explicit Init_TractionDriveCommand_speed(::hqv_public_interface::msg::TractionDriveCommand & msg)
  : msg_(msg)
  {}
  Init_TractionDriveCommand_steering speed(::hqv_public_interface::msg::TractionDriveCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_TractionDriveCommand_steering(msg_);
  }

private:
  ::hqv_public_interface::msg::TractionDriveCommand msg_;
};

class Init_TractionDriveCommand_header
{
public:
  Init_TractionDriveCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TractionDriveCommand_speed header(::hqv_public_interface::msg::TractionDriveCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TractionDriveCommand_speed(msg_);
  }

private:
  ::hqv_public_interface::msg::TractionDriveCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::TractionDriveCommand>()
{
  return hqv_public_interface::msg::builder::Init_TractionDriveCommand_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__TRACTION_DRIVE_COMMAND__BUILDER_HPP_
