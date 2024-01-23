// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverMoveTo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_co_driver_move_to__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerCoDriverMoveTo_y
{
public:
  explicit Init_MowerCoDriverMoveTo_y(::hqv_public_interface::msg::MowerCoDriverMoveTo & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerCoDriverMoveTo y(::hqv_public_interface::msg::MowerCoDriverMoveTo::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverMoveTo msg_;
};

class Init_MowerCoDriverMoveTo_x
{
public:
  Init_MowerCoDriverMoveTo_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerCoDriverMoveTo_y x(::hqv_public_interface::msg::MowerCoDriverMoveTo::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MowerCoDriverMoveTo_y(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverMoveTo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerCoDriverMoveTo>()
{
  return hqv_public_interface::msg::builder::Init_MowerCoDriverMoveTo_x();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__BUILDER_HPP_
