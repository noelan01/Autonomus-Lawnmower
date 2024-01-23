// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverCommand.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_COMMAND__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_COMMAND__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_co_driver_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerCoDriverCommand_command
{
public:
  explicit Init_MowerCoDriverCommand_command(::hqv_public_interface::msg::MowerCoDriverCommand & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerCoDriverCommand command(::hqv_public_interface::msg::MowerCoDriverCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverCommand msg_;
};

class Init_MowerCoDriverCommand_header
{
public:
  Init_MowerCoDriverCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerCoDriverCommand_command header(::hqv_public_interface::msg::MowerCoDriverCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerCoDriverCommand_command(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerCoDriverCommand>()
{
  return hqv_public_interface::msg::builder::Init_MowerCoDriverCommand_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_COMMAND__BUILDER_HPP_
