// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/HwMon.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/hw_mon__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::msg::HwMon & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: cpu_freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpu_freq: ";
    value_to_yaml(msg.cpu_freq, out);
    out << "\n";
  }

  // member: used_memory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "used_memory: ";
    value_to_yaml(msg.used_memory, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::HwMon & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::HwMon>()
{
  return "hqv_public_interface::msg::HwMon";
}

template<>
inline const char * name<hqv_public_interface::msg::HwMon>()
{
  return "hqv_public_interface/msg/HwMon";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::HwMon>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::HwMon>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::msg::HwMon>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__TRAITS_HPP_
