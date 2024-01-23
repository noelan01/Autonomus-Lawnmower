// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGuideSignal.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_guide_signal__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGuideSignal_g3
{
public:
  explicit Init_MowerGuideSignal_g3(::hqv_public_interface::msg::MowerGuideSignal & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGuideSignal g3(::hqv_public_interface::msg::MowerGuideSignal::_g3_type arg)
  {
    msg_.g3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGuideSignal msg_;
};

class Init_MowerGuideSignal_g2
{
public:
  explicit Init_MowerGuideSignal_g2(::hqv_public_interface::msg::MowerGuideSignal & msg)
  : msg_(msg)
  {}
  Init_MowerGuideSignal_g3 g2(::hqv_public_interface::msg::MowerGuideSignal::_g2_type arg)
  {
    msg_.g2 = std::move(arg);
    return Init_MowerGuideSignal_g3(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGuideSignal msg_;
};

class Init_MowerGuideSignal_g1
{
public:
  explicit Init_MowerGuideSignal_g1(::hqv_public_interface::msg::MowerGuideSignal & msg)
  : msg_(msg)
  {}
  Init_MowerGuideSignal_g2 g1(::hqv_public_interface::msg::MowerGuideSignal::_g1_type arg)
  {
    msg_.g1 = std::move(arg);
    return Init_MowerGuideSignal_g2(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGuideSignal msg_;
};

class Init_MowerGuideSignal_quality
{
public:
  explicit Init_MowerGuideSignal_quality(::hqv_public_interface::msg::MowerGuideSignal & msg)
  : msg_(msg)
  {}
  Init_MowerGuideSignal_g1 quality(::hqv_public_interface::msg::MowerGuideSignal::_quality_type arg)
  {
    msg_.quality = std::move(arg);
    return Init_MowerGuideSignal_g1(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGuideSignal msg_;
};

class Init_MowerGuideSignal_index
{
public:
  explicit Init_MowerGuideSignal_index(::hqv_public_interface::msg::MowerGuideSignal & msg)
  : msg_(msg)
  {}
  Init_MowerGuideSignal_quality index(::hqv_public_interface::msg::MowerGuideSignal::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_MowerGuideSignal_quality(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGuideSignal msg_;
};

class Init_MowerGuideSignal_header
{
public:
  Init_MowerGuideSignal_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGuideSignal_index header(::hqv_public_interface::msg::MowerGuideSignal::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGuideSignal_index(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGuideSignal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGuideSignal>()
{
  return hqv_public_interface::msg::builder::Init_MowerGuideSignal_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__BUILDER_HPP_
