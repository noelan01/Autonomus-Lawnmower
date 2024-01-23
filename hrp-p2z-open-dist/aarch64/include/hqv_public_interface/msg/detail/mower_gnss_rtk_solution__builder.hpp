// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerGnssRtkSolution.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_SOLUTION__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_SOLUTION__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_rtk_solution__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerGnssRtkSolution_solution
{
public:
  explicit Init_MowerGnssRtkSolution_solution(::hqv_public_interface::msg::MowerGnssRtkSolution & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerGnssRtkSolution solution(::hqv_public_interface::msg::MowerGnssRtkSolution::_solution_type arg)
  {
    msg_.solution = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkSolution msg_;
};

class Init_MowerGnssRtkSolution_header
{
public:
  Init_MowerGnssRtkSolution_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerGnssRtkSolution_solution header(::hqv_public_interface::msg::MowerGnssRtkSolution::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MowerGnssRtkSolution_solution(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerGnssRtkSolution msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerGnssRtkSolution>()
{
  return hqv_public_interface::msg::builder::Init_MowerGnssRtkSolution_header();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_SOLUTION__BUILDER_HPP_
