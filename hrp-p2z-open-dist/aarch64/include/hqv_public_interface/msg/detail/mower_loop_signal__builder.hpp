// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerLoopSignal.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LOOP_SIGNAL__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LOOP_SIGNAL__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_loop_signal__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerLoopSignal_n
{
public:
  explicit Init_MowerLoopSignal_n(::hqv_public_interface::msg::MowerLoopSignal & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerLoopSignal n(::hqv_public_interface::msg::MowerLoopSignal::_n_type arg)
  {
    msg_.n = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLoopSignal msg_;
};

class Init_MowerLoopSignal_f
{
public:
  explicit Init_MowerLoopSignal_f(::hqv_public_interface::msg::MowerLoopSignal & msg)
  : msg_(msg)
  {}
  Init_MowerLoopSignal_n f(::hqv_public_interface::msg::MowerLoopSignal::_f_type arg)
  {
    msg_.f = std::move(arg);
    return Init_MowerLoopSignal_n(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLoopSignal msg_;
};

class Init_MowerLoopSignal_a
{
public:
  explicit Init_MowerLoopSignal_a(::hqv_public_interface::msg::MowerLoopSignal & msg)
  : msg_(msg)
  {}
  Init_MowerLoopSignal_f a(::hqv_public_interface::msg::MowerLoopSignal::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_MowerLoopSignal_f(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLoopSignal msg_;
};

class Init_MowerLoopSignal_inside
{
public:
  explicit Init_MowerLoopSignal_inside(::hqv_public_interface::msg::MowerLoopSignal & msg)
  : msg_(msg)
  {}
  Init_MowerLoopSignal_a inside(::hqv_public_interface::msg::MowerLoopSignal::_inside_type arg)
  {
    msg_.inside = std::move(arg);
    return Init_MowerLoopSignal_a(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLoopSignal msg_;
};

class Init_MowerLoopSignal_index
{
public:
  explicit Init_MowerLoopSignal_index(::hqv_public_interface::msg::MowerLoopSignal & msg)
  : msg_(msg)
  {}
  Init_MowerLoopSignal_inside index(::hqv_public_interface::msg::MowerLoopSignal::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_MowerLoopSignal_inside(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLoopSignal msg_;
};

class Init_MowerLoopSignal_header
{
public:
  Init_MowerLoopSignal_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerLoopSignal_index header(::hqv_public_interface::msg::MowerLoopSignal::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerLoopSignal_index(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLoopSignal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerLoopSignal>()
{
  return hqv_public_interface::msg::builder::Init_MowerLoopSignal_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LOOP_SIGNAL__BUILDER_HPP_
