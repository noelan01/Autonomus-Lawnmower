// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/TifServiceCall.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/tif_service_call__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_TifServiceCall_Request_timeout
{
public:
  explicit Init_TifServiceCall_Request_timeout(::hqv_public_interface::srv::TifServiceCall_Request & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::TifServiceCall_Request timeout(::hqv_public_interface::srv::TifServiceCall_Request::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::TifServiceCall_Request msg_;
};

class Init_TifServiceCall_Request_payload
{
public:
  explicit Init_TifServiceCall_Request_payload(::hqv_public_interface::srv::TifServiceCall_Request & msg)
  : msg_(msg)
  {}
  Init_TifServiceCall_Request_timeout payload(::hqv_public_interface::srv::TifServiceCall_Request::_payload_type arg)
  {
    msg_.payload = std::move(arg);
    return Init_TifServiceCall_Request_timeout(msg_);
  }

private:
  ::hqv_public_interface::srv::TifServiceCall_Request msg_;
};

class Init_TifServiceCall_Request_command
{
public:
  explicit Init_TifServiceCall_Request_command(::hqv_public_interface::srv::TifServiceCall_Request & msg)
  : msg_(msg)
  {}
  Init_TifServiceCall_Request_payload command(::hqv_public_interface::srv::TifServiceCall_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_TifServiceCall_Request_payload(msg_);
  }

private:
  ::hqv_public_interface::srv::TifServiceCall_Request msg_;
};

class Init_TifServiceCall_Request_family
{
public:
  explicit Init_TifServiceCall_Request_family(::hqv_public_interface::srv::TifServiceCall_Request & msg)
  : msg_(msg)
  {}
  Init_TifServiceCall_Request_command family(::hqv_public_interface::srv::TifServiceCall_Request::_family_type arg)
  {
    msg_.family = std::move(arg);
    return Init_TifServiceCall_Request_command(msg_);
  }

private:
  ::hqv_public_interface::srv::TifServiceCall_Request msg_;
};

class Init_TifServiceCall_Request_nodename
{
public:
  Init_TifServiceCall_Request_nodename()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TifServiceCall_Request_family nodename(::hqv_public_interface::srv::TifServiceCall_Request::_nodename_type arg)
  {
    msg_.nodename = std::move(arg);
    return Init_TifServiceCall_Request_family(msg_);
  }

private:
  ::hqv_public_interface::srv::TifServiceCall_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::TifServiceCall_Request>()
{
  return hqv_public_interface::srv::builder::Init_TifServiceCall_Request_nodename();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_TifServiceCall_Response_data
{
public:
  explicit Init_TifServiceCall_Response_data(::hqv_public_interface::srv::TifServiceCall_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::TifServiceCall_Response data(::hqv_public_interface::srv::TifServiceCall_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::TifServiceCall_Response msg_;
};

class Init_TifServiceCall_Response_result
{
public:
  Init_TifServiceCall_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TifServiceCall_Response_data result(::hqv_public_interface::srv::TifServiceCall_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_TifServiceCall_Response_data(msg_);
  }

private:
  ::hqv_public_interface::srv::TifServiceCall_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::TifServiceCall_Response>()
{
  return hqv_public_interface::srv::builder::Init_TifServiceCall_Response_result();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__BUILDER_HPP_
