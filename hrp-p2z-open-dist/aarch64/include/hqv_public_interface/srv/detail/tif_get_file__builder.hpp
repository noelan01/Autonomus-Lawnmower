// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/TifGetFile.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/tif_get_file__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_TifGetFile_Request_filename
{
public:
  Init_TifGetFile_Request_filename()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::srv::TifGetFile_Request filename(::hqv_public_interface::srv::TifGetFile_Request::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::TifGetFile_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::TifGetFile_Request>()
{
  return hqv_public_interface::srv::builder::Init_TifGetFile_Request_filename();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_TifGetFile_Response_data
{
public:
  explicit Init_TifGetFile_Response_data(::hqv_public_interface::srv::TifGetFile_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::TifGetFile_Response data(::hqv_public_interface::srv::TifGetFile_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::TifGetFile_Response msg_;
};

class Init_TifGetFile_Response_status
{
public:
  Init_TifGetFile_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TifGetFile_Response_data status(::hqv_public_interface::srv::TifGetFile_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_TifGetFile_Response_data(msg_);
  }

private:
  ::hqv_public_interface::srv::TifGetFile_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::TifGetFile_Response>()
{
  return hqv_public_interface::srv::builder::Init_TifGetFile_Response_status();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__BUILDER_HPP_
