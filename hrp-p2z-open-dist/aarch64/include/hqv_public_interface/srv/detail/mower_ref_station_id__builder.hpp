// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:srv/MowerRefStationId.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_ID__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_ID__BUILDER_HPP_

#include "hqv_public_interface/srv/detail/mower_ref_station_id__struct.hpp"
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
auto build<::hqv_public_interface::srv::MowerRefStationId_Request>()
{
  return ::hqv_public_interface::srv::MowerRefStationId_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace srv
{

namespace builder
{

class Init_MowerRefStationId_Response_station_id
{
public:
  explicit Init_MowerRefStationId_Response_station_id(::hqv_public_interface::srv::MowerRefStationId_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::srv::MowerRefStationId_Response station_id(::hqv_public_interface::srv::MowerRefStationId_Response::_station_id_type arg)
  {
    msg_.station_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerRefStationId_Response msg_;
};

class Init_MowerRefStationId_Response_status
{
public:
  Init_MowerRefStationId_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerRefStationId_Response_station_id status(::hqv_public_interface::srv::MowerRefStationId_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MowerRefStationId_Response_station_id(msg_);
  }

private:
  ::hqv_public_interface::srv::MowerRefStationId_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::srv::MowerRefStationId_Response>()
{
  return hqv_public_interface::srv::builder::Init_MowerRefStationId_Response_status();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_ID__BUILDER_HPP_
