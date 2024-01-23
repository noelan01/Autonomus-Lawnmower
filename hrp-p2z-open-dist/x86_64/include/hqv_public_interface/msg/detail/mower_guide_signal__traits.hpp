// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MowerGuideSignal.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/mower_guide_signal__struct.hpp"
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
  const hqv_public_interface::msg::MowerGuideSignal & msg,
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

  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index: ";
    value_to_yaml(msg.index, out);
    out << "\n";
  }

  // member: quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quality: ";
    value_to_yaml(msg.quality, out);
    out << "\n";
  }

  // member: g1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g1: ";
    value_to_yaml(msg.g1, out);
    out << "\n";
  }

  // member: g2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g2: ";
    value_to_yaml(msg.g2, out);
    out << "\n";
  }

  // member: g3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g3: ";
    value_to_yaml(msg.g3, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MowerGuideSignal & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MowerGuideSignal>()
{
  return "hqv_public_interface::msg::MowerGuideSignal";
}

template<>
inline const char * name<hqv_public_interface::msg::MowerGuideSignal>()
{
  return "hqv_public_interface/msg/MowerGuideSignal";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MowerGuideSignal>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MowerGuideSignal>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hqv_public_interface::msg::MowerGuideSignal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GUIDE_SIGNAL__TRAITS_HPP_
