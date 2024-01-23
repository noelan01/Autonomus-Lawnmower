// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/WheelPair.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/wheel_pair__struct.hpp"
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
  const hqv_public_interface::msg::WheelPair & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }

  // member: power_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power_left: ";
    value_to_yaml(msg.power_left, out);
    out << "\n";
  }

  // member: power_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power_right: ";
    value_to_yaml(msg.power_right, out);
    out << "\n";
  }

  // member: counter_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "counter_left: ";
    value_to_yaml(msg.counter_left, out);
    out << "\n";
  }

  // member: counter_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "counter_right: ";
    value_to_yaml(msg.counter_right, out);
    out << "\n";
  }

  // member: free_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "free_left: ";
    value_to_yaml(msg.free_left, out);
    out << "\n";
  }

  // member: free_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "free_right: ";
    value_to_yaml(msg.free_right, out);
    out << "\n";
  }

  // member: deltadistance_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deltadistance_left: ";
    value_to_yaml(msg.deltadistance_left, out);
    out << "\n";
  }

  // member: deltadistance_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "deltadistance_right: ";
    value_to_yaml(msg.deltadistance_right, out);
    out << "\n";
  }

  // member: state_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_left: ";
    value_to_yaml(msg.state_left, out);
    out << "\n";
  }

  // member: state_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_right: ";
    value_to_yaml(msg.state_right, out);
    out << "\n";
  }

  // member: current_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_left: ";
    value_to_yaml(msg.current_left, out);
    out << "\n";
  }

  // member: current_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_right: ";
    value_to_yaml(msg.current_right, out);
    out << "\n";
  }

  // member: speed_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_left: ";
    value_to_yaml(msg.speed_left, out);
    out << "\n";
  }

  // member: speed_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_right: ";
    value_to_yaml(msg.speed_right, out);
    out << "\n";
  }

  // member: isrunning_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "isrunning_left: ";
    value_to_yaml(msg.isrunning_left, out);
    out << "\n";
  }

  // member: isrunning_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "isrunning_right: ";
    value_to_yaml(msg.isrunning_right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::WheelPair & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::WheelPair>()
{
  return "hqv_public_interface::msg::WheelPair";
}

template<>
inline const char * name<hqv_public_interface::msg::WheelPair>()
{
  return "hqv_public_interface/msg/WheelPair";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::WheelPair>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::WheelPair>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hqv_public_interface::msg::WheelPair>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__TRAITS_HPP_
