// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerImuYaw.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_IMU_YAW__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_IMU_YAW__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_imu_yaw__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerImuYaw_yaw
{
public:
  explicit Init_MowerImuYaw_yaw(::hqv_public_interface::msg::MowerImuYaw & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerImuYaw yaw(::hqv_public_interface::msg::MowerImuYaw::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerImuYaw msg_;
};

class Init_MowerImuYaw_header
{
public:
  Init_MowerImuYaw_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerImuYaw_yaw header(::hqv_public_interface::msg::MowerImuYaw::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerImuYaw_yaw(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerImuYaw msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerImuYaw>()
{
  return hqv_public_interface::msg::builder::Init_MowerImuYaw_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_IMU_YAW__BUILDER_HPP_
