// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerMowerMission.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MISSION__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MISSION__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_mower_mission__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerMowerMission_description
{
public:
  explicit Init_MowerMowerMission_description(::hqv_public_interface::msg::MowerMowerMission & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerMowerMission description(::hqv_public_interface::msg::MowerMowerMission::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerMission msg_;
};

class Init_MowerMowerMission_mission
{
public:
  explicit Init_MowerMowerMission_mission(::hqv_public_interface::msg::MowerMowerMission & msg)
  : msg_(msg)
  {}
  Init_MowerMowerMission_description mission(::hqv_public_interface::msg::MowerMowerMission::_mission_type arg)
  {
    msg_.mission = std::move(arg);
    return Init_MowerMowerMission_description(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerMission msg_;
};

class Init_MowerMowerMission_header
{
public:
  Init_MowerMowerMission_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerMowerMission_mission header(::hqv_public_interface::msg::MowerMowerMission::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerMowerMission_mission(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerMowerMission msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerMowerMission>()
{
  return hqv_public_interface::msg::builder::Init_MowerMowerMission_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MISSION__BUILDER_HPP_
