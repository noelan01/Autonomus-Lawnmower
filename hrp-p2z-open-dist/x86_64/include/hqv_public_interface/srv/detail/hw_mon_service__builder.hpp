// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/HwMonService.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/hw_mon_service__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::HwMonService_Request>()
{
  return ::hqv_public_interface::srv::HwMonService_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_HwMonService_Response_hw_monitor
{
public:
  Init_HwMonService_Response_hw_monitor()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::srv::HwMonService_Response hw_monitor(::hqv_public_interface::srv::HwMonService_Response::_hw_monitor_type arg)
  {
    msg_.hw_monitor = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::HwMonService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::HwMonService_Response>()
{
  return hqv_public_interface::srv::builder::Init_HwMonService_Response_hw_monitor();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__BUILDER_HPP_
