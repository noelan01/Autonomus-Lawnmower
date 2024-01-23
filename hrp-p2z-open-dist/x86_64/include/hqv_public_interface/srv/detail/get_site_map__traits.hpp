// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:srv/GetSiteMap.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__TRAITS_HPP_

#include "hqv_public_interface/srv/detail/get_site_map__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::srv::GetSiteMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request: ";
    value_to_yaml(msg.request, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::srv::GetSiteMap_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::srv::GetSiteMap_Request>()
{
  return "hqv_public_interface::srv::GetSiteMap_Request";
}

template<>
inline const char * name<hqv_public_interface::srv::GetSiteMap_Request>()
{
  return "hqv_public_interface/srv/GetSiteMap_Request";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::GetSiteMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::srv::GetSiteMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::srv::GetSiteMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::srv::GetSiteMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: loaded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loaded: ";
    value_to_yaml(msg.loaded, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: sitename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sitename: ";
    value_to_yaml(msg.sitename, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        character_value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::srv::GetSiteMap_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::srv::GetSiteMap_Response>()
{
  return "hqv_public_interface::srv::GetSiteMap_Response";
}

template<>
inline const char * name<hqv_public_interface::srv::GetSiteMap_Response>()
{
  return "hqv_public_interface/srv/GetSiteMap_Response";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::GetSiteMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hqv_public_interface::srv::GetSiteMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hqv_public_interface::srv::GetSiteMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hqv_public_interface::srv::GetSiteMap>()
{
  return "hqv_public_interface::srv::GetSiteMap";
}

template<>
inline const char * name<hqv_public_interface::srv::GetSiteMap>()
{
  return "hqv_public_interface/srv/GetSiteMap";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::GetSiteMap>
  : std::integral_constant<
    bool,
    has_fixed_size<hqv_public_interface::srv::GetSiteMap_Request>::value &&
    has_fixed_size<hqv_public_interface::srv::GetSiteMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<hqv_public_interface::srv::GetSiteMap>
  : std::integral_constant<
    bool,
    has_bounded_size<hqv_public_interface::srv::GetSiteMap_Request>::value &&
    has_bounded_size<hqv_public_interface::srv::GetSiteMap_Response>::value
  >
{
};

template<>
struct is_service<hqv_public_interface::srv::GetSiteMap>
  : std::true_type
{
};

template<>
struct is_service_request<hqv_public_interface::srv::GetSiteMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hqv_public_interface::srv::GetSiteMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__TRAITS_HPP_
