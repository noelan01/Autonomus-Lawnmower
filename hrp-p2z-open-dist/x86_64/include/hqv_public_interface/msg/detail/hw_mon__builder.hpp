// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/HwMon.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/hw_mon__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_HwMon_used_memory
{
public:
  explicit Init_HwMon_used_memory(::hqv_public_interface::msg::HwMon & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::HwMon used_memory(::hqv_public_interface::msg::HwMon::_used_memory_type arg)
  {
    msg_.used_memory = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::HwMon msg_;
};

class Init_HwMon_cpu_freq
{
public:
  explicit Init_HwMon_cpu_freq(::hqv_public_interface::msg::HwMon & msg)
  : msg_(msg)
  {}
  Init_HwMon_used_memory cpu_freq(::hqv_public_interface::msg::HwMon::_cpu_freq_type arg)
  {
    msg_.cpu_freq = std::move(arg);
    return Init_HwMon_used_memory(msg_);
  }

private:
  ::hqv_public_interface::msg::HwMon msg_;
};

class Init_HwMon_temperature
{
public:
  Init_HwMon_temperature()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HwMon_cpu_freq temperature(::hqv_public_interface::msg::HwMon::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_HwMon_cpu_freq(msg_);
  }

private:
  ::hqv_public_interface::msg::HwMon msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::HwMon>()
{
  return hqv_public_interface::msg::builder::Init_HwMon_temperature();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__BUILDER_HPP_
