// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:srv/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__TRAITS_HPP_

#include "hqv_public_interface/srv/detail/mower_ref_station_position__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::srv::MowerRefStationPosition_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: message_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message_type: ";
    value_to_yaml(msg.message_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::srv::MowerRefStationPosition_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::srv::MowerRefStationPosition_Request>()
{
  return "hqv_public_interface::srv::MowerRefStationPosition_Request";
}

template<>
inline const char * name<hqv_public_interface::srv::MowerRefStationPosition_Request>()
{
  return "hqv_public_interface/srv/MowerRefStationPosition_Request";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::MowerRefStationPosition_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::srv::MowerRefStationPosition_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::srv::MowerRefStationPosition_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::srv::MowerRefStationPosition_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    value_to_yaml(msg.status, out);
    out << "\n";
  }

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

inline std::string to_yaml(const hqv_public_interface::srv::MowerRefStationPosition_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::srv::MowerRefStationPosition_Response>()
{
  return "hqv_public_interface::srv::MowerRefStationPosition_Response";
}

template<>
inline const char * name<hqv_public_interface::srv::MowerRefStationPosition_Response>()
{
  return "hqv_public_interface/srv/MowerRefStationPosition_Response";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::MowerRefStationPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::srv::MowerRefStationPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::srv::MowerRefStationPosition_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hqv_public_interface::srv::MowerRefStationPosition>()
{
  return "hqv_public_interface::srv::MowerRefStationPosition";
}

template<>
inline const char * name<hqv_public_interface::srv::MowerRefStationPosition>()
{
  return "hqv_public_interface/srv/MowerRefStationPosition";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::MowerRefStationPosition>
  : std::integral_constant<
    bool,
    has_fixed_size<hqv_public_interface::srv::MowerRefStationPosition_Request>::value &&
    has_fixed_size<hqv_public_interface::srv::MowerRefStationPosition_Response>::value
  >
{
};

template<>
struct has_bounded_size<hqv_public_interface::srv::MowerRefStationPosition>
  : std::integral_constant<
    bool,
    has_bounded_size<hqv_public_interface::srv::MowerRefStationPosition_Request>::value &&
    has_bounded_size<hqv_public_interface::srv::MowerRefStationPosition_Response>::value
  >
{
};

template<>
struct is_service<hqv_public_interface::srv::MowerRefStationPosition>
  : std::true_type
{
};

template<>
struct is_service_request<hqv_public_interface::srv::MowerRefStationPosition_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hqv_public_interface::srv::MowerRefStationPosition_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__TRAITS_HPP_
