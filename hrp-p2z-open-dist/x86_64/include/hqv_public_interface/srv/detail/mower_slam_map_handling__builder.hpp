// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/MowerSlamMapHandling.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/mower_slam_map_handling__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_MowerSlamMapHandling_Request_map_id
{
public:
  explicit Init_MowerSlamMapHandling_Request_map_id(::hqv_public_interface::srv::MowerSlamMapHandling_Request & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::MowerSlamMapHandling_Request map_id(::hqv_public_interface::srv::MowerSlamMapHandling_Request::_map_id_type arg)
  {
    msg_.map_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerSlamMapHandling_Request msg_;
};

class Init_MowerSlamMapHandling_Request_message_type
{
public:
  Init_MowerSlamMapHandling_Request_message_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerSlamMapHandling_Request_map_id message_type(::hqv_public_interface::srv::MowerSlamMapHandling_Request::_message_type_type arg)
  {
    msg_.message_type = std::move(arg);
    return Init_MowerSlamMapHandling_Request_map_id(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerSlamMapHandling_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::MowerSlamMapHandling_Request>()
{
  return hqv_public_interface::srv::builder::Init_MowerSlamMapHandling_Request_message_type();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_MowerSlamMapHandling_Response_map_ids
{
public:
  explicit Init_MowerSlamMapHandling_Response_map_ids(::hqv_public_interface::srv::MowerSlamMapHandling_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::MowerSlamMapHandling_Response map_ids(::hqv_public_interface::srv::MowerSlamMapHandling_Response::_map_ids_type arg)
  {
    msg_.map_ids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerSlamMapHandling_Response msg_;
};

class Init_MowerSlamMapHandling_Response_map_id
{
public:
  Init_MowerSlamMapHandling_Response_map_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerSlamMapHandling_Response_map_ids map_id(::hqv_public_interface::srv::MowerSlamMapHandling_Response::_map_id_type arg)
  {
    msg_.map_id = std::move(arg);
    return Init_MowerSlamMapHandling_Response_map_ids(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerSlamMapHandling_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::MowerSlamMapHandling_Response>()
{
  return hqv_public_interface::srv::builder::Init_MowerSlamMapHandling_Response_map_id();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__BUILDER_HPP_
