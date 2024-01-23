// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssState.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssState_state
{
public:
  explicit Init_MowerGnssState_state(::hqv_public_interface::msg::MowerGnssState & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssState state(::hqv_public_interface::msg::MowerGnssState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssState msg_;
};

class Init_MowerGnssState_header
{
public:
  Init_MowerGnssState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssState_state header(::hqv_public_interface::msg::MowerGnssState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssState_state(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssState>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssState_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__BUILDER_HPP_
