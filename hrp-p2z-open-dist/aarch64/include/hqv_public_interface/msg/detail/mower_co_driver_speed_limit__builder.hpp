// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverSpeedLimit.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_SPEED_LIMIT__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_SPEED_LIMIT__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_co_driver_speed_limit__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerCoDriverSpeedLimit_speedlimit
{
public:
  Init_MowerCoDriverSpeedLimit_speedlimit()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::msg::MowerCoDriverSpeedLimit speedlimit(::hqv_public_interface::msg::MowerCoDriverSpeedLimit::_speedlimit_type arg)
  {
    msg_.speedlimit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverSpeedLimit msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerCoDriverSpeedLimit>()
{
  return hqv_public_interface::msg::builder::Init_MowerCoDriverSpeedLimit_speedlimit();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_SPEED_LIMIT__BUILDER_HPP_
