// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:srv/GetMap.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__TRAITS_HPP_

#include "hqv_public_interface/srv/detail/get_map__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::srv::GetMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::srv::GetMap_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::srv::GetMap_Request>()
{
  return "hqv_public_interface::srv::GetMap_Request";
}

template<>
inline const char * name<hqv_public_interface::srv::GetMap_Request>()
{
  return "hqv_public_interface/srv/GetMap_Request";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::GetMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::srv::GetMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::srv::GetMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'map_info'
#include "hqv_public_interface/msg/detail/map_info__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::srv::GetMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    value_to_yaml(msg.result, out);
    out << "\n";
  }

  // member: map_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_info:\n";
    to_yaml(msg.map_info, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::srv::GetMap_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::srv::GetMap_Response>()
{
  return "hqv_public_interface::srv::GetMap_Response";
}

template<>
inline const char * name<hqv_public_interface::srv::GetMap_Response>()
{
  return "hqv_public_interface/srv/GetMap_Response";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::GetMap_Response>
  : std::integral_constant<bool, has_fixed_size<hqv_public_interface::msg::MapInfo>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::srv::GetMap_Response>
  : std::integral_constant<bool, has_bounded_size<hqv_public_interface::msg::MapInfo>::value> {};

template<>
struct is_message<hqv_public_interface::srv::GetMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hqv_public_interface::srv::GetMap>()
{
  return "hqv_public_interface::srv::GetMap";
}

template<>
inline const char * name<hqv_public_interface::srv::GetMap>()
{
  return "hqv_public_interface/srv/GetMap";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::GetMap>
  : std::integral_constant<
    bool,
    has_fixed_size<hqv_public_interface::srv::GetMap_Request>::value &&
    has_fixed_size<hqv_public_interface::srv::GetMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<hqv_public_interface::srv::GetMap>
  : std::integral_constant<
    bool,
    has_bounded_size<hqv_public_interface::srv::GetMap_Request>::value &&
    has_bounded_size<hqv_public_interface::srv::GetMap_Response>::value
  >
{
};

template<>
struct is_service<hqv_public_interface::srv::GetMap>
  : std::true_type
{
};

template<>
struct is_service_request<hqv_public_interface::srv::GetMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hqv_public_interface::srv::GetMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_MAP__TRAITS_HPP_
