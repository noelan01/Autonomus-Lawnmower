// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerSlamHandling.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_slam_handling__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerSlamHandling_slam_handle
{
public:
  Init_MowerSlamHandling_slam_handle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::msg::MowerSlamHandling slam_handle(::hqv_public_interface::msg::MowerSlamHandling::_slam_handle_type arg)
  {
    msg_.slam_handle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSlamHandling msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerSlamHandling>()
{
  return hqv_public_interface::msg::builder::Init_MowerSlamHandling_slam_handle();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__BUILDER_HPP_
