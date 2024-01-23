// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MowerLedStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/mower_led_status__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::msg::MowerLedStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mower_spp_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mower_spp_state: ";
    value_to_yaml(msg.mower_spp_state, out);
    out << "\n";
  }

  // member: mower_app_activity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mower_app_activity: ";
    value_to_yaml(msg.mower_app_activity, out);
    out << "\n";
  }

  // member: extended_status_host_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "extended_status_host_message: ";
    value_to_yaml(msg.extended_status_host_message, out);
    out << "\n";
  }

  // member: operation_manager_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation_manager_state: ";
    value_to_yaml(msg.operation_manager_state, out);
    out << "\n";
  }

  // member: in_charging_station
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_charging_station: ";
    value_to_yaml(msg.in_charging_station, out);
    out << "\n";
  }

  // member: is_stopped
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_stopped: ";
    value_to_yaml(msg.is_stopped, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    value_to_yaml(msg.color, out);
    out << "\n";
  }

  // member: led_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led_mode: ";
    value_to_yaml(msg.led_mode, out);
    out << "\n";
  }

  // member: period
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "period: ";
    value_to_yaml(msg.period, out);
    out << "\n";
  }

  // member: log_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "log_id: ";
    value_to_yaml(msg.log_id, out);
    out << "\n";
  }

  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MowerLedStatus & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MowerLedStatus>()
{
  return "hqv_public_interface::msg::MowerLedStatus";
}

template<>
inline const char * name<hqv_public_interface::msg::MowerLedStatus>()
{
  return "hqv_public_interface/msg/MowerLedStatus";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MowerLedStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MowerLedStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hqv_public_interface::msg::MowerLedStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__TRAITS_HPP_
