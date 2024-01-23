// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverRotate.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_ROTATE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_ROTATE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_co_driver_rotate__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerCoDriverRotate_rotateangle
{
public:
  Init_MowerCoDriverRotate_rotateangle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::msg::MowerCoDriverRotate rotateangle(::hqv_public_interface::msg::MowerCoDriverRotate::_rotateangle_type arg)
  {
    msg_.rotateangle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCoDriverRotate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerCoDriverRotate>()
{
  return hqv_public_interface::msg::builder::Init_MowerCoDriverRotate_rotateangle();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_ROTATE__BUILDER_HPP_
