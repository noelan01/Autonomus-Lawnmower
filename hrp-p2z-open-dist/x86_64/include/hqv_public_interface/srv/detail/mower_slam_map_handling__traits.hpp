// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:srv/MowerSlamMapHandling.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__TRAITS_HPP_

#include "hqv_public_interface/srv/detail/mower_slam_map_handling__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::srv::MowerSlamMapHandling_Request & msg,
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

  // member: map_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_id: ";
    value_to_yaml(msg.map_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::srv::MowerSlamMapHandling_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::srv::MowerSlamMapHandling_Request>()
{
  return "hqv_public_interface::srv::MowerSlamMapHandling_Request";
}

template<>
inline const char * name<hqv_public_interface::srv::MowerSlamMapHandling_Request>()
{
  return "hqv_public_interface/srv/MowerSlamMapHandling_Request";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::MowerSlamMapHandling_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::srv::MowerSlamMapHandling_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::srv::MowerSlamMapHandling_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::srv::MowerSlamMapHandling_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_id: ";
    value_to_yaml(msg.map_id, out);
    out << "\n";
  }

  // member: map_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.map_ids.size() == 0) {
      out << "map_ids: []\n";
    } else {
      out << "map_ids:\n";
      for (auto item : msg.map_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::srv::MowerSlamMapHandling_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::srv::MowerSlamMapHandling_Response>()
{
  return "hqv_public_interface::srv::MowerSlamMapHandling_Response";
}

template<>
inline const char * name<hqv_public_interface::srv::MowerSlamMapHandling_Response>()
{
  return "hqv_public_interface/srv/MowerSlamMapHandling_Response";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::MowerSlamMapHandling_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::srv::MowerSlamMapHandling_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::srv::MowerSlamMapHandling_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hqv_public_interface::srv::MowerSlamMapHandling>()
{
  return "hqv_public_interface::srv::MowerSlamMapHandling";
}

template<>
inline const char * name<hqv_public_interface::srv::MowerSlamMapHandling>()
{
  return "hqv_public_interface/srv/MowerSlamMapHandling";
}

template<>
struct has_fixed_size<hqv_public_interface::srv::MowerSlamMapHandling>
  : std::integral_constant<
    bool,
    has_fixed_size<hqv_public_interface::srv::MowerSlamMapHandling_Request>::value &&
    has_fixed_size<hqv_public_interface::srv::MowerSlamMapHandling_Response>::value
  >
{
};

template<>
struct has_bounded_size<hqv_public_interface::srv::MowerSlamMapHandling>
  : std::integral_constant<
    bool,
    has_bounded_size<hqv_public_interface::srv::MowerSlamMapHandling_Request>::value &&
    has_bounded_size<hqv_public_interface::srv::MowerSlamMapHandling_Response>::value
  >
{
};

template<>
struct is_service<hqv_public_interface::srv::MowerSlamMapHandling>
  : std::true_type
{
};

template<>
struct is_service_request<hqv_public_interface::srv::MowerSlamMapHandling_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hqv_public_interface::srv::MowerSlamMapHandling_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__TRAITS_HPP_
