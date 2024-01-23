// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/LocalPositionStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__LOCAL_POSITION_STATUS__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__LOCAL_POSITION_STATUS__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/local_position_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_LocalPositionStatus_reliable
{
public:
  explicit Init_LocalPositionStatus_reliable(::hqv_public_interface::msg::LocalPositionStatus & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::LocalPositionStatus reliable(::hqv_public_interface::msg::LocalPositionStatus::_reliable_type arg)
  {
    msg_.reliable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::LocalPositionStatus msg_;
};

class Init_LocalPositionStatus_ready
{
public:
  explicit Init_LocalPositionStatus_ready(::hqv_public_interface::msg::LocalPositionStatus & msg)
  : msg_(msg)
  {}
  Init_LocalPositionStatus_reliable ready(::hqv_public_interface::msg::LocalPositionStatus::_ready_type arg)
  {
    msg_.ready = std::move(arg);
    return Init_LocalPositionStatus_reliable(msg_);
  }

private:
  ::hqv_public_interface::msg::LocalPositionStatus msg_;
};

class Init_LocalPositionStatus_header
{
public:
  Init_LocalPositionStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalPositionStatus_ready header(::hqv_public_interface::msg::LocalPositionStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LocalPositionStatus_ready(msg_);
  }

private:
  ::hqv_public_interface::msg::LocalPositionStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::LocalPositionStatus>()
{
  return hqv_public_interface::msg::builder::Init_LocalPositionStatus_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__LOCAL_POSITION_STATUS__BUILDER_HPP_
