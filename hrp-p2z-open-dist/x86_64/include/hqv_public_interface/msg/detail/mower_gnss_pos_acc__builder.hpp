// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssPosAcc.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POS_ACC__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POS_ACC__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_pos_acc__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssPosAcc_vertical
{
public:
  explicit Init_MowerGnssPosAcc_vertical(::hqv_public_interface::msg::MowerGnssPosAcc & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssPosAcc vertical(::hqv_public_interface::msg::MowerGnssPosAcc::_vertical_type arg)
  {
    msg_.vertical = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPosAcc msg_;
};

class Init_MowerGnssPosAcc_horizontal
{
public:
  explicit Init_MowerGnssPosAcc_horizontal(::hqv_public_interface::msg::MowerGnssPosAcc & msg)
  : msg_(msg)
  {}
  Init_MowerGnssPosAcc_vertical horizontal(::hqv_public_interface::msg::MowerGnssPosAcc::_horizontal_type arg)
  {
    msg_.horizontal = std::move(arg);
    return Init_MowerGnssPosAcc_vertical(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPosAcc msg_;
};

class Init_MowerGnssPosAcc_header
{
public:
  Init_MowerGnssPosAcc_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssPosAcc_horizontal header(::hqv_public_interface::msg::MowerGnssPosAcc::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssPosAcc_horizontal(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssPosAcc msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssPosAcc>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssPosAcc_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POS_ACC__BUILDER_HPP_
