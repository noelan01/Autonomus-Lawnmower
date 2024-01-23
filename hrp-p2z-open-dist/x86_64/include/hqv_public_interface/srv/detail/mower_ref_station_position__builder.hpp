// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/mower_ref_station_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_MowerRefStationPosition_Request_message_type
{
public:
  Init_MowerRefStationPosition_Request_message_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::srv::MowerRefStationPosition_Request message_type(::hqv_public_interface::srv::MowerRefStationPosition_Request::_message_type_type arg)
  {
    msg_.message_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerRefStationPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::MowerRefStationPosition_Request>()
{
  return hqv_public_interface::srv::builder::Init_MowerRefStationPosition_Request_message_type();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_MowerRefStationPosition_Response_altitude
{
public:
  explicit Init_MowerRefStationPosition_Response_altitude(::hqv_public_interface::srv::MowerRefStationPosition_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::MowerRefStationPosition_Response altitude(::hqv_public_interface::srv::MowerRefStationPosition_Response::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerRefStationPosition_Response msg_;
};

class Init_MowerRefStationPosition_Response_latitude
{
public:
  explicit Init_MowerRefStationPosition_Response_latitude(::hqv_public_interface::srv::MowerRefStationPosition_Response & msg)
  : msg_(msg)
  {}
  Init_MowerRefStationPosition_Response_altitude latitude(::hqv_public_interface::srv::MowerRefStationPosition_Response::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_MowerRefStationPosition_Response_altitude(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerRefStationPosition_Response msg_;
};

class Init_MowerRefStationPosition_Response_longitude
{
public:
  explicit Init_MowerRefStationPosition_Response_longitude(::hqv_public_interface::srv::MowerRefStationPosition_Response & msg)
  : msg_(msg)
  {}
  Init_MowerRefStationPosition_Response_latitude longitude(::hqv_public_interface::srv::MowerRefStationPosition_Response::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_MowerRefStationPosition_Response_latitude(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerRefStationPosition_Response msg_;
};

class Init_MowerRefStationPosition_Response_status
{
public:
  Init_MowerRefStationPosition_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerRefStationPosition_Response_longitude status(::hqv_public_interface::srv::MowerRefStationPosition_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MowerRefStationPosition_Response_longitude(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerRefStationPosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::MowerRefStationPosition_Response>()
{
  return hqv_public_interface::srv::builder::Init_MowerRefStationPosition_Response_status();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__BUILDER_HPP_
