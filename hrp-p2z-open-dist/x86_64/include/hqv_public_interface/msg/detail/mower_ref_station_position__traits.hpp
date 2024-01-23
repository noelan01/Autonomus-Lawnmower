// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/mower_ref_station_position__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::msg::MowerRefStationPosition & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    value_to_yaml(msg.altitude, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MowerRefStationPosition & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MowerRefStationPosition>()
{
  return "hqv_public_interface::msg::MowerRefStationPosition";
}

template<>
inline const char * name<hqv_public_interface::msg::MowerRefStationPosition>()
{
  return "hqv_public_interface/msg/MowerRefStationPosition";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MowerRefStationPosition>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MowerRefStationPosition>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::msg::MowerRefStationPosition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_REF_STATION_POSITION__TRAITS_HPP_
