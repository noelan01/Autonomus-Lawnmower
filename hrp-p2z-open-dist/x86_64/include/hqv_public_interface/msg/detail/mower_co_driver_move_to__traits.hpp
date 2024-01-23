// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MowerCoDriverMoveTo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/mower_co_driver_move_to__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::msg::MowerCoDriverMoveTo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MowerCoDriverMoveTo & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MowerCoDriverMoveTo>()
{
  return "hqv_public_interface::msg::MowerCoDriverMoveTo";
}

template<>
inline const char * name<hqv_public_interface::msg::MowerCoDriverMoveTo>()
{
  return "hqv_public_interface/msg/MowerCoDriverMoveTo";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MowerCoDriverMoveTo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MowerCoDriverMoveTo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::msg::MowerCoDriverMoveTo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__TRAITS_HPP_
