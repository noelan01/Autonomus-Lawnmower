// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:msg/MowerLedStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__BUILDER_HPP_

#include "hqv_public_interface/msg/detail/mower_led_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace msg
{

namespace builder
{

class Init_MowerLedStatus_header
{
public:
  explicit Init_MowerLedStatus_header(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::msg::MowerLedStatus header(::hqv_public_interface::msg::MowerLedStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_log_id
{
public:
  explicit Init_MowerLedStatus_log_id(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_header log_id(::hqv_public_interface::msg::MowerLedStatus::_log_id_type arg)
  {
    msg_.log_id = std::move(arg);
    return Init_MowerLedStatus_header(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_period
{
public:
  explicit Init_MowerLedStatus_period(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_log_id period(::hqv_public_interface::msg::MowerLedStatus::_period_type arg)
  {
    msg_.period = std::move(arg);
    return Init_MowerLedStatus_log_id(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_led_mode
{
public:
  explicit Init_MowerLedStatus_led_mode(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_period led_mode(::hqv_public_interface::msg::MowerLedStatus::_led_mode_type arg)
  {
    msg_.led_mode = std::move(arg);
    return Init_MowerLedStatus_period(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_color
{
public:
  explicit Init_MowerLedStatus_color(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_led_mode color(::hqv_public_interface::msg::MowerLedStatus::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_MowerLedStatus_led_mode(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_is_stopped
{
public:
  explicit Init_MowerLedStatus_is_stopped(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_color is_stopped(::hqv_public_interface::msg::MowerLedStatus::_is_stopped_type arg)
  {
    msg_.is_stopped = std::move(arg);
    return Init_MowerLedStatus_color(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_in_charging_station
{
public:
  explicit Init_MowerLedStatus_in_charging_station(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_is_stopped in_charging_station(::hqv_public_interface::msg::MowerLedStatus::_in_charging_station_type arg)
  {
    msg_.in_charging_station = std::move(arg);
    return Init_MowerLedStatus_is_stopped(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_operation_manager_state
{
public:
  explicit Init_MowerLedStatus_operation_manager_state(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_in_charging_station operation_manager_state(::hqv_public_interface::msg::MowerLedStatus::_operation_manager_state_type arg)
  {
    msg_.operation_manager_state = std::move(arg);
    return Init_MowerLedStatus_in_charging_station(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_extended_status_host_message
{
public:
  explicit Init_MowerLedStatus_extended_status_host_message(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_operation_manager_state extended_status_host_message(::hqv_public_interface::msg::MowerLedStatus::_extended_status_host_message_type arg)
  {
    msg_.extended_status_host_message = std::move(arg);
    return Init_MowerLedStatus_operation_manager_state(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_mower_app_activity
{
public:
  explicit Init_MowerLedStatus_mower_app_activity(::hqv_public_interface::msg::MowerLedStatus & msg)
  : msg_(msg)
  {}
  Init_MowerLedStatus_extended_status_host_message mower_app_activity(::hqv_public_interface::msg::MowerLedStatus::_mower_app_activity_type arg)
  {
    msg_.mower_app_activity = std::move(arg);
    return Init_MowerLedStatus_extended_status_host_message(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

class Init_MowerLedStatus_mower_spp_state
{
public:
  Init_MowerLedStatus_mower_spp_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerLedStatus_mower_app_activity mower_spp_state(::hqv_public_interface::msg::MowerLedStatus::_mower_spp_state_type arg)
  {
    msg_.mower_spp_state = std::move(arg);
    return Init_MowerLedStatus_mower_app_activity(msg_);
  }

private:
  ::hqv_public_interface::msg::MowerLedStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::msg::MowerLedStatus>()
{
  return hqv_public_interface::msg::builder::Init_MowerLedStatus_mower_spp_state();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__BUILDER_HPP_
