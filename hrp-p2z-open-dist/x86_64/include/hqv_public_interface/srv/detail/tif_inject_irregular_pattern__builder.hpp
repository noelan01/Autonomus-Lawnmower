// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/TifInjectIrregularPattern.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_TifInjectIrregularPattern_Request_set_state
{
public:
  Init_TifInjectIrregularPattern_Request_set_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::srv::TifInjectIrregularPattern_Request set_state(::hqv_public_interface::srv::TifInjectIrregularPattern_Request::_set_state_type arg)
  {
    msg_.set_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::TifInjectIrregularPattern_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::TifInjectIrregularPattern_Request>()
{
  return hqv_public_interface::srv::builder::Init_TifInjectIrregularPattern_Request_set_state();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_TifInjectIrregularPattern_Response_status
{
public:
  Init_TifInjectIrregularPattern_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::srv::TifInjectIrregularPattern_Response status(::hqv_public_interface::srv::TifInjectIrregularPattern_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::TifInjectIrregularPattern_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::TifInjectIrregularPattern_Response>()
{
  return hqv_public_interface::srv::builder::Init_TifInjectIrregularPattern_Response_status();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__BUILDER_HPP_
