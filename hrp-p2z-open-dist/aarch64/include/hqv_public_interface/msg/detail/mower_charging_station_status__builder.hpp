// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerChargingStationStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CHARGING_STATION_STATUS__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CHARGING_STATION_STATUS__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_charging_station_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerChargingStationStatus_mower_in_cs
{
public:
  Init_MowerChargingStationStatus_mower_in_cs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::msg::MowerChargingStationStatus mower_in_cs(::hqv_public_interface::msg::MowerChargingStationStatus::_mower_in_cs_type arg)
  {
    msg_.mower_in_cs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerChargingStationStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerChargingStationStatus>()
{
  return hqv_public_interface::msg::builder::Init_MowerChargingStationStatus_mower_in_cs();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CHARGING_STATION_STATUS__BUILDER_HPP_
