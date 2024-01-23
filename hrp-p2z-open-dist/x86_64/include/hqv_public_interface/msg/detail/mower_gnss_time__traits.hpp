// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MowerGnssTime.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_time__struct.hpp"
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
  const hqv_public_interface::msg::MowerGnssTime & msg,
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

  // member: itow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "itow: ";
    value_to_yaml(msg.itow, out);
    out << "\n";
  }

  // member: year
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "year: ";
    value_to_yaml(msg.year, out);
    out << "\n";
  }

  // member: month
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "month: ";
    value_to_yaml(msg.month, out);
    out << "\n";
  }

  // member: day
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "day: ";
    value_to_yaml(msg.day, out);
    out << "\n";
  }

  // member: hour
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hour: ";
    value_to_yaml(msg.hour, out);
    out << "\n";
  }

  // member: minute
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "minute: ";
    value_to_yaml(msg.minute, out);
    out << "\n";
  }

  // member: second
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "second: ";
    value_to_yaml(msg.second, out);
    out << "\n";
  }

  // member: nano
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nano: ";
    value_to_yaml(msg.nano, out);
    out << "\n";
  }

  // member: timeaccuracy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeaccuracy: ";
    value_to_yaml(msg.timeaccuracy, out);
    out << "\n";
  }

  // member: timevalid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timevalid: ";
    value_to_yaml(msg.timevalid, out);
    out << "\n";
  }

  // member: datevalid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "datevalid: ";
    value_to_yaml(msg.datevalid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MowerGnssTime & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MowerGnssTime>()
{
  return "hqv_public_interface::msg::MowerGnssTime";
}

template<>
inline const char * name<hqv_public_interface::msg::MowerGnssTime>()
{
  return "hqv_public_interface/msg/MowerGnssTime";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MowerGnssTime>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MowerGnssTime>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hqv_public_interface::msg::MowerGnssTime>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__TRAITS_HPP_
