// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelativePositionENU.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_rtk_relative_position_enu__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssRtkRelativePositionENU_accuracy_up
{
public:
  explicit Init_MowerGnssRtkRelativePositionENU_accuracy_up(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU accuracy_up(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU::_accuracy_up_type arg)
  {
    msg_.accuracy_up = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU msg_;
};

class Init_MowerGnssRtkRelativePositionENU_accuracy_north
{
public:
  explicit Init_MowerGnssRtkRelativePositionENU_accuracy_north(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelativePositionENU_accuracy_up accuracy_north(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU::_accuracy_north_type arg)
  {
    msg_.accuracy_north = std::move(arg);
    return Init_MowerGnssRtkRelativePositionENU_accuracy_up(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU msg_;
};

class Init_MowerGnssRtkRelativePositionENU_accuracy_east
{
public:
  explicit Init_MowerGnssRtkRelativePositionENU_accuracy_east(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelativePositionENU_accuracy_north accuracy_east(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU::_accuracy_east_type arg)
  {
    msg_.accuracy_east = std::move(arg);
    return Init_MowerGnssRtkRelativePositionENU_accuracy_north(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU msg_;
};

class Init_MowerGnssRtkRelativePositionENU_up
{
public:
  explicit Init_MowerGnssRtkRelativePositionENU_up(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelativePositionENU_accuracy_east up(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU::_up_type arg)
  {
    msg_.up = std::move(arg);
    return Init_MowerGnssRtkRelativePositionENU_accuracy_east(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU msg_;
};

class Init_MowerGnssRtkRelativePositionENU_north
{
public:
  explicit Init_MowerGnssRtkRelativePositionENU_north(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelativePositionENU_up north(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU::_north_type arg)
  {
    msg_.north = std::move(arg);
    return Init_MowerGnssRtkRelativePositionENU_up(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU msg_;
};

class Init_MowerGnssRtkRelativePositionENU_east
{
public:
  explicit Init_MowerGnssRtkRelativePositionENU_east(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU & msg)
  : msg_(msg)
  {}
  Init_MowerGnssRtkRelativePositionENU_north east(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU::_east_type arg)
  {
    msg_.east = std::move(arg);
    return Init_MowerGnssRtkRelativePositionENU_north(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU msg_;
};

class Init_MowerGnssRtkRelativePositionENU_header
{
public:
  Init_MowerGnssRtkRelativePositionENU_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssRtkRelativePositionENU_east header(::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssRtkRelativePositionENU_east(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssRtkRelativePositionENU>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssRtkRelativePositionENU_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__BUILDER_HPP_
