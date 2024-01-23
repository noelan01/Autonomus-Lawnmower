// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerLift.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_lift__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerLift_status
{
public:
  explicit Init_MowerLift_status(::hqv_public_interface::msg::MowerLift & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerLift status(::hqv_public_interface::msg::MowerLift::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLift msg_;
};

class Init_MowerLift_sensors
{
public:
  explicit Init_MowerLift_sensors(::hqv_public_interface::msg::MowerLift & msg)
  : msg_(msg)
  {}
  Init_MowerLift_status sensors(::hqv_public_interface::msg::MowerLift::_sensors_type arg)
  {
    msg_.sensors = std::move(arg);
    return Init_MowerLift_status(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLift msg_;
};

class Init_MowerLift_lift
{
public:
  explicit Init_MowerLift_lift(::hqv_public_interface::msg::MowerLift & msg)
  : msg_(msg)
  {}
  Init_MowerLift_sensors lift(::hqv_public_interface::msg::MowerLift::_lift_type arg)
  {
    msg_.lift = std::move(arg);
    return Init_MowerLift_sensors(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLift msg_;
};

class Init_MowerLift_header
{
public:
  Init_MowerLift_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerLift_lift header(::hqv_public_interface::msg::MowerLift::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerLift_lift(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLift msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerLift>()
{
  return hqv_public_interface::msg::builder::Init_MowerLift_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT__BUILDER_HPP_
