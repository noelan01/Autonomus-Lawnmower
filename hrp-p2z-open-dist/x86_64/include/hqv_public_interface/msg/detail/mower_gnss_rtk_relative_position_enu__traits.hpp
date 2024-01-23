// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelativePositionENU.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_rtk_relative_position_enu__struct.hpp"
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
  const hqv_public_interface::msg::MowerGnssRtkRelativePositionENU & msg,
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

  // member: east
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "east: ";
    value_to_yaml(msg.east, out);
    out << "\n";
  }

  // member: north
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "north: ";
    value_to_yaml(msg.north, out);
    out << "\n";
  }

  // member: up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "up: ";
    value_to_yaml(msg.up, out);
    out << "\n";
  }

  // member: accuracy_east
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accuracy_east: ";
    value_to_yaml(msg.accuracy_east, out);
    out << "\n";
  }

  // member: accuracy_north
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accuracy_north: ";
    value_to_yaml(msg.accuracy_north, out);
    out << "\n";
  }

  // member: accuracy_up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accuracy_up: ";
    value_to_yaml(msg.accuracy_up, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MowerGnssRtkRelativePositionENU & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU>()
{
  return "hqv_public_interface::msg::MowerGnssRtkRelativePositionENU";
}

template<>
inline const char * name<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU>()
{
  return "hqv_public_interface/msg/MowerGnssRtkRelativePositionENU";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hqv_public_interface::msg::MowerGnssRtkRelativePositionENU>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_RELATIVE_POSITION_ENU__TRAITS_HPP_
