// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverResult.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_co_driver_result__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerCoDriverResult_resultcode
{
public:
  explicit Init_MowerCoDriverResult_resultcode(::hqv_public_interface::msg::MowerCoDriverResult & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerCoDriverResult resultcode(::hqv_public_interface::msg::MowerCoDriverResult::_resultcode_type arg)
  {
    msg_.resultcode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverResult msg_;
};

class Init_MowerCoDriverResult_command
{
public:
  explicit Init_MowerCoDriverResult_command(::hqv_public_interface::msg::MowerCoDriverResult & msg)
  : msg_(msg)
  {}
  Init_MowerCoDriverResult_resultcode command(::hqv_public_interface::msg::MowerCoDriverResult::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_MowerCoDriverResult_resultcode(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverResult msg_;
};

class Init_MowerCoDriverResult_result
{
public:
  explicit Init_MowerCoDriverResult_result(::hqv_public_interface::msg::MowerCoDriverResult & msg)
  : msg_(msg)
  {}
  Init_MowerCoDriverResult_command result(::hqv_public_interface::msg::MowerCoDriverResult::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_MowerCoDriverResult_command(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverResult msg_;
};

class Init_MowerCoDriverResult_header
{
public:
  Init_MowerCoDriverResult_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerCoDriverResult_result header(::hqv_public_interface::msg::MowerCoDriverResult::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerCoDriverResult_result(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerCoDriverResult>()
{
  return hqv_public_interface::msg::builder::Init_MowerCoDriverResult_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__BUILDER_HPP_
