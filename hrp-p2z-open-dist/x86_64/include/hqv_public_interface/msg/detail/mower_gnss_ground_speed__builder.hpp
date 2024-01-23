// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssGroundSpeed.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_GROUND_SPEED__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_GROUND_SPEED__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_ground_speed__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssGroundSpeed_acc
{
public:
  explicit Init_MowerGnssGroundSpeed_acc(::hqv_public_interface::msg::MowerGnssGroundSpeed & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssGroundSpeed acc(::hqv_public_interface::msg::MowerGnssGroundSpeed::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssGroundSpeed msg_;
};

class Init_MowerGnssGroundSpeed_speed
{
public:
  explicit Init_MowerGnssGroundSpeed_speed(::hqv_public_interface::msg::MowerGnssGroundSpeed & msg)
  : msg_(msg)
  {}
  Init_MowerGnssGroundSpeed_acc speed(::hqv_public_interface::msg::MowerGnssGroundSpeed::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_MowerGnssGroundSpeed_acc(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssGroundSpeed msg_;
};

class Init_MowerGnssGroundSpeed_header
{
public:
  Init_MowerGnssGroundSpeed_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssGroundSpeed_speed header(::hqv_public_interface::msg::MowerGnssGroundSpeed::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssGroundSpeed_speed(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssGroundSpeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssGroundSpeed>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssGroundSpeed_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_GROUND_SPEED__BUILDER_HPP_
