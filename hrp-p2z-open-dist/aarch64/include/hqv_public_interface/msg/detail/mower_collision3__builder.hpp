// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerCollision3.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_COLLISION3__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_COLLISION3__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_collision3__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerCollision3_rearright
{
public:
  explicit Init_MowerCollision3_rearright(::hqv_public_interface::msg::MowerCollision3 & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerCollision3 rearright(::hqv_public_interface::msg::MowerCollision3::_rearright_type arg)
  {
    msg_.rearright = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCollision3 msg_;
};

class Init_MowerCollision3_rearleft
{
public:
  explicit Init_MowerCollision3_rearleft(::hqv_public_interface::msg::MowerCollision3 & msg)
  : msg_(msg)
  {}
  Init_MowerCollision3_rearright rearleft(::hqv_public_interface::msg::MowerCollision3::_rearleft_type arg)
  {
    msg_.rearleft = std::move(arg);
    return Init_MowerCollision3_rearright(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCollision3 msg_;
};

class Init_MowerCollision3_frontcenter
{
public:
  explicit Init_MowerCollision3_frontcenter(::hqv_public_interface::msg::MowerCollision3 & msg)
  : msg_(msg)
  {}
  Init_MowerCollision3_rearleft frontcenter(::hqv_public_interface::msg::MowerCollision3::_frontcenter_type arg)
  {
    msg_.frontcenter = std::move(arg);
    return Init_MowerCollision3_rearleft(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCollision3 msg_;
};

class Init_MowerCollision3_header
{
public:
  Init_MowerCollision3_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerCollision3_frontcenter header(::hqv_public_interface::msg::MowerCollision3::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerCollision3_frontcenter(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerCollision3 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerCollision3>()
{
  return hqv_public_interface::msg::builder::Init_MowerCollision3_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_COLLISION3__BUILDER_HPP_
