// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_ref_station_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerRefStationPosition_altitude
{
public:
  explicit Init_MowerRefStationPosition_altitude(::hqv_public_interface::msg::MowerRefStationPosition & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerRefStationPosition altitude(::hqv_public_interface::msg::MowerRefStationPosition::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerRefStationPosition msg_;
};

class Init_MowerRefStationPosition_latitude
{
public:
  explicit Init_MowerRefStationPosition_latitude(::hqv_public_interface::msg::MowerRefStationPosition & msg)
  : msg_(msg)
  {}
  Init_MowerRefStationPosition_altitude latitude(::hqv_public_interface::msg::MowerRefStationPosition::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_MowerRefStationPosition_altitude(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerRefStationPosition msg_;
};

class Init_MowerRefStationPosition_longitude
{
public:
  Init_MowerRefStationPosition_longitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerRefStationPosition_latitude longitude(::hqv_public_interface::msg::MowerRefStationPosition::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_MowerRefStationPosition_latitude(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerRefStationPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerRefStationPosition>()
{
  return hqv_public_interface::msg::builder::Init_MowerRefStationPosition_longitude();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__BUILDER_HPP_
