// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerMowerError.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ERROR__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ERROR__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_mower_error__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerMowerError_confirmable
{
public:
  explicit Init_MowerMowerError_confirmable(::hqv_public_interface::msg::MowerMowerError & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerMowerError confirmable(::hqv_public_interface::msg::MowerMowerError::_confirmable_type arg)
  {
    msg_.confirmable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerError msg_;
};

class Init_MowerMowerError_error_code
{
public:
  explicit Init_MowerMowerError_error_code(::hqv_public_interface::msg::MowerMowerError & msg)
  : msg_(msg)
  {}
  Init_MowerMowerError_confirmable error_code(::hqv_public_interface::msg::MowerMowerError::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_MowerMowerError_confirmable(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerError msg_;
};

class Init_MowerMowerError_header
{
public:
  Init_MowerMowerError_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerMowerError_error_code header(::hqv_public_interface::msg::MowerMowerError::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerMowerError_error_code(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerError msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerMowerError>()
{
  return hqv_public_interface::msg::builder::Init_MowerMowerError_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ERROR__BUILDER_HPP_
