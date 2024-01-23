// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/GetSiteMap.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/get_site_map__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_GetSiteMap_Request_request
{
public:
  Init_GetSiteMap_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::srv::GetSiteMap_Request request(::hqv_public_interface::srv::GetSiteMap_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::GetSiteMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::GetSiteMap_Request>()
{
  return hqv_public_interface::srv::builder::Init_GetSiteMap_Request_request();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_GetSiteMap_Response_data
{
public:
  explicit Init_GetSiteMap_Response_data(::hqv_public_interface::srv::GetSiteMap_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::GetSiteMap_Response data(::hqv_public_interface::srv::GetSiteMap_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::GetSiteMap_Response msg_;
};

class Init_GetSiteMap_Response_sitename
{
public:
  explicit Init_GetSiteMap_Response_sitename(::hqv_public_interface::srv::GetSiteMap_Response & msg)
  : msg_(msg)
  {}
  Init_GetSiteMap_Response_data sitename(::hqv_public_interface::srv::GetSiteMap_Response::_sitename_type arg)
  {
    msg_.sitename = std::move(arg);
    return Init_GetSiteMap_Response_data(msg_);
  }

private:
  ::hqv_public_interface::srv::GetSiteMap_Response msg_;
};

class Init_GetSiteMap_Response_valid
{
public:
  explicit Init_GetSiteMap_Response_valid(::hqv_public_interface::srv::GetSiteMap_Response & msg)
  : msg_(msg)
  {}
  Init_GetSiteMap_Response_sitename valid(::hqv_public_interface::srv::GetSiteMap_Response::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_GetSiteMap_Response_sitename(msg_);
  }

private:
  ::hqv_public_interface::srv::GetSiteMap_Response msg_;
};

class Init_GetSiteMap_Response_loaded
{
public:
  Init_GetSiteMap_Response_loaded()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSiteMap_Response_valid loaded(::hqv_public_interface::srv::GetSiteMap_Response::_loaded_type arg)
  {
    msg_.loaded = std::move(arg);
    return Init_GetSiteMap_Response_valid(msg_);
  }

private:
  ::hqv_public_interface::srv::GetSiteMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::GetSiteMap_Response>()
{
  return hqv_public_interface::srv::builder::Init_GetSiteMap_Response_loaded();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__BUILDER_HPP_
