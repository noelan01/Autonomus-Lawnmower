// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerLiftSensor2.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT_SENSOR2__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT_SENSOR2__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_lift_sensor2__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerLiftSensor2_frontright
{
public:
  explicit Init_MowerLiftSensor2_frontright(::hqv_public_interface::msg::MowerLiftSensor2 & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerLiftSensor2 frontright(::hqv_public_interface::msg::MowerLiftSensor2::_frontright_type arg)
  {
    msg_.frontright = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLiftSensor2 msg_;
};

class Init_MowerLiftSensor2_frontleft
{
public:
  explicit Init_MowerLiftSensor2_frontleft(::hqv_public_interface::msg::MowerLiftSensor2 & msg)
  : msg_(msg)
  {}
  Init_MowerLiftSensor2_frontright frontleft(::hqv_public_interface::msg::MowerLiftSensor2::_frontleft_type arg)
  {
    msg_.frontleft = std::move(arg);
    return Init_MowerLiftSensor2_frontright(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLiftSensor2 msg_;
};

class Init_MowerLiftSensor2_header
{
public:
  Init_MowerLiftSensor2_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerLiftSensor2_frontleft header(::hqv_public_interface::msg::MowerLiftSensor2::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerLiftSensor2_frontleft(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLiftSensor2 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerLiftSensor2>()
{
  return hqv_public_interface::msg::builder::Init_MowerLiftSensor2_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT_SENSOR2__BUILDER_HPP_
