// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerAcc.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_ACC__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_ACC__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_acc__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerAcc_hwtime
{
public:
  explicit Init_MowerAcc_hwtime(::hqv_public_interface::msg::MowerAcc & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerAcc hwtime(::hqv_public_interface::msg::MowerAcc::_hwtime_type arg)
  {
    msg_.hwtime = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerAcc msg_;
};

class Init_MowerAcc_z
{
public:
  explicit Init_MowerAcc_z(::hqv_public_interface::msg::MowerAcc & msg)
  : msg_(msg)
  {}
  Init_MowerAcc_hwtime z(::hqv_public_interface::msg::MowerAcc::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_MowerAcc_hwtime(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerAcc msg_;
};

class Init_MowerAcc_y
{
public:
  explicit Init_MowerAcc_y(::hqv_public_interface::msg::MowerAcc & msg)
  : msg_(msg)
  {}
  Init_MowerAcc_z y(::hqv_public_interface::msg::MowerAcc::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MowerAcc_z(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerAcc msg_;
};

class Init_MowerAcc_x
{
public:
  explicit Init_MowerAcc_x(::hqv_public_interface::msg::MowerAcc & msg)
  : msg_(msg)
  {}
  Init_MowerAcc_y x(::hqv_public_interface::msg::MowerAcc::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MowerAcc_y(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerAcc msg_;
};

class Init_MowerAcc_header
{
public:
  Init_MowerAcc_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerAcc_x header(::hqv_public_interface::msg::MowerAcc::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerAcc_x(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerAcc msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerAcc>()
{
  return hqv_public_interface::msg::builder::Init_MowerAcc_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_ACC__BUILDER_HPP_
