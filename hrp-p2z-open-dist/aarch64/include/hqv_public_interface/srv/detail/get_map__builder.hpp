// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/GetMap.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/get_map__struct.hpp"
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
auto build<::hqv_public_interface::srv::GetMap_Request>()
{
  return ::hqv_public_interface::srv::GetMap_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_GetMap_Response_map_info
{
public:
  explicit Init_GetMap_Response_map_info(::hqv_public_interface::srv::GetMap_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::GetMap_Response map_info(::hqv_public_interface::srv::GetMap_Response::_map_info_type arg)
  {
    msg_.map_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::GetMap_Response msg_;
};

class Init_GetMap_Response_result
{
public:
  Init_GetMap_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetMap_Response_map_info result(::hqv_public_interface::srv::GetMap_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_GetMap_Response_map_info(msg_);
  }

private:
  ::hqv_public_interface::srv::GetMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::GetMap_Response>()
{
  return hqv_public_interface::srv::builder::Init_GetMap_Response_result();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__BUILDER_HPP_
