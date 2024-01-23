// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssHeading.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HEADING__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HEADING__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_heading__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssHeading_acc
{
public:
  explicit Init_MowerGnssHeading_acc(::hqv_public_interface::msg::MowerGnssHeading & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssHeading acc(::hqv_public_interface::msg::MowerGnssHeading::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssHeading msg_;
};

class Init_MowerGnssHeading_mower
{
public:
  explicit Init_MowerGnssHeading_mower(::hqv_public_interface::msg::MowerGnssHeading & msg)
  : msg_(msg)
  {}
  Init_MowerGnssHeading_acc mower(::hqv_public_interface::msg::MowerGnssHeading::_mower_type arg)
  {
    msg_.mower = std::move(arg);
    return Init_MowerGnssHeading_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssHeading msg_;
};

class Init_MowerGnssHeading_motion
{
public:
  explicit Init_MowerGnssHeading_motion(::hqv_public_interface::msg::MowerGnssHeading & msg)
  : msg_(msg)
  {}
  Init_MowerGnssHeading_mower motion(::hqv_public_interface::msg::MowerGnssHeading::_motion_type arg)
  {
    msg_.motion = std::move(arg);
    return Init_MowerGnssHeading_mower(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssHeading msg_;
};

class Init_MowerGnssHeading_header
{
public:
  Init_MowerGnssHeading_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssHeading_motion header(::hqv_public_interface::msg::MowerGnssHeading::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssHeading_motion(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssHeading msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssHeading>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssHeading_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HEADING__BUILDER_HPP_
