// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerCollision.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_COLLISION__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_COLLISION__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_collision__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerCollision_collision
{
public:
  explicit Init_MowerCollision_collision(::hqv_public_interface::msg::MowerCollision & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerCollision collision(::hqv_public_interface::msg::MowerCollision::_collision_type arg)
  {
    msg_.collision = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCollision msg_;
};

class Init_MowerCollision_header
{
public:
  Init_MowerCollision_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerCollision_collision header(::hqv_public_interface::msg::MowerCollision::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerCollision_collision(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCollision msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerCollision>()
{
  return hqv_public_interface::msg::builder::Init_MowerCollision_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_COLLISION__BUILDER_HPP_
