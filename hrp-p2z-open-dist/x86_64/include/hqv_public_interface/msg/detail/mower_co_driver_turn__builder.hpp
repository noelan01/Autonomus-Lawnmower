// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverTurn.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_TURN__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_TURN__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_co_driver_turn__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerCoDriverTurn_turnangle
{
public:
  Init_MowerCoDriverTurn_turnangle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::msg::MowerCoDriverTurn turnangle(::hqv_public_interface::msg::MowerCoDriverTurn::_turnangle_type arg)
  {
    msg_.turnangle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverTurn msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerCoDriverTurn>()
{
  return hqv_public_interface::msg::builder::Init_MowerCoDriverTurn_turnangle();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_TURN__BUILDER_HPP_
