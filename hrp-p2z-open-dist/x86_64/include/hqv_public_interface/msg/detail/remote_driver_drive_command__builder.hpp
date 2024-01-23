// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/RemoteDriverDriveCommand.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__REMOTE_DRIVER_DRIVE_COMMAND__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__REMOTE_DRIVER_DRIVE_COMMAND__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/remote_driver_drive_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_RemoteDriverDriveCommand_steering
{
public:
  explicit Init_RemoteDriverDriveCommand_steering(::hqv_public_interface::msg::RemoteDriverDriveCommand & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::RemoteDriverDriveCommand steering(::hqv_public_interface::msg::RemoteDriverDriveCommand::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::RemoteDriverDriveCommand msg_;
};

class Init_RemoteDriverDriveCommand_speed
{
public:
  explicit Init_RemoteDriverDriveCommand_speed(::hqv_public_interface::msg::RemoteDriverDriveCommand & msg)
  : msg_(msg)
  {}
  Init_RemoteDriverDriveCommand_steering speed(::hqv_public_interface::msg::RemoteDriverDriveCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_RemoteDriverDriveCommand_steering(msg_);
  }

private:
  ::hqv_public_interface::msg::RemoteDriverDriveCommand msg_;
};

class Init_RemoteDriverDriveCommand_header
{
public:
  Init_RemoteDriverDriveCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RemoteDriverDriveCommand_speed header(::hqv_public_interface::msg::RemoteDriverDriveCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RemoteDriverDriveCommand_speed(msg_);
  }

private:
  ::hqv_public_interface::msg::RemoteDriverDriveCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::RemoteDriverDriveCommand>()
{
  return hqv_public_interface::msg::builder::Init_RemoteDriverDriveCommand_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__REMOTE_DRIVER_DRIVE_COMMAND__BUILDER_HPP_
