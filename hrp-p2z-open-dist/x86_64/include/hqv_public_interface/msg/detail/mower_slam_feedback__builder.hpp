// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerSlamFeedback.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_slam_feedback__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerSlamFeedback_feedback_data_2
{
public:
  explicit Init_MowerSlamFeedback_feedback_data_2(::hqv_public_interface::msg::MowerSlamFeedback & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerSlamFeedback feedback_data_2(::hqv_public_interface::msg::MowerSlamFeedback::_feedback_data_2_type arg)
  {
    msg_.feedback_data_2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSlamFeedback msg_;
};

class Init_MowerSlamFeedback_feedback_data_1
{
public:
  explicit Init_MowerSlamFeedback_feedback_data_1(::hqv_public_interface::msg::MowerSlamFeedback & msg)
  : msg_(msg)
  {}
  Init_MowerSlamFeedback_feedback_data_2 feedback_data_1(::hqv_public_interface::msg::MowerSlamFeedback::_feedback_data_1_type arg)
  {
    msg_.feedback_data_1 = std::move(arg);
    return Init_MowerSlamFeedback_feedback_data_2(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSlamFeedback msg_;
};

class Init_MowerSlamFeedback_feedback
{
public:
  Init_MowerSlamFeedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerSlamFeedback_feedback_data_1 feedback(::hqv_public_interface::msg::MowerSlamFeedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return Init_MowerSlamFeedback_feedback_data_1(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerSlamFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerSlamFeedback>()
{
  return hqv_public_interface::msg::builder::Init_MowerSlamFeedback_feedback();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__BUILDER_HPP_
