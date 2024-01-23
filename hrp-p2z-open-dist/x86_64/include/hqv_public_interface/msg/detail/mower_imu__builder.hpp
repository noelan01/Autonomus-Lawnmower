// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerImu.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_IMU__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_IMU__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_imu__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerImu_hwtime
{
public:
  explicit Init_MowerImu_hwtime(::hqv_public_interface::msg::MowerImu & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerImu hwtime(::hqv_public_interface::msg::MowerImu::_hwtime_type arg)
  {
    msg_.hwtime = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerImu msg_;
};

class Init_MowerImu_yaw
{
public:
  explicit Init_MowerImu_yaw(::hqv_public_interface::msg::MowerImu & msg)
  : msg_(msg)
  {}
  Init_MowerImu_hwtime yaw(::hqv_public_interface::msg::MowerImu::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_MowerImu_hwtime(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerImu msg_;
};

class Init_MowerImu_pitch
{
public:
  explicit Init_MowerImu_pitch(::hqv_public_interface::msg::MowerImu & msg)
  : msg_(msg)
  {}
  Init_MowerImu_yaw pitch(::hqv_public_interface::msg::MowerImu::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_MowerImu_yaw(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerImu msg_;
};

class Init_MowerImu_roll
{
public:
  explicit Init_MowerImu_roll(::hqv_public_interface::msg::MowerImu & msg)
  : msg_(msg)
  {}
  Init_MowerImu_pitch roll(::hqv_public_interface::msg::MowerImu::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_MowerImu_pitch(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerImu msg_;
};

class Init_MowerImu_header
{
public:
  Init_MowerImu_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerImu_roll header(::hqv_public_interface::msg::MowerImu::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerImu_roll(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerImu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerImu>()
{
  return hqv_public_interface::msg::builder::Init_MowerImu_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_IMU__BUILDER_HPP_
