// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MapShape.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/map_shape__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::msg::MapShape & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: polygon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "polygon:\n";
    to_yaml(msg.polygon, out, indentation + 2);
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.id.size() == 0) {
      out << "id: []\n";
    } else {
      out << "id:\n";
      for (auto item : msg.id) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: stroke_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stroke_color: ";
    value_to_yaml(msg.stroke_color, out);
    out << "\n";
  }

  // member: fill_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fill_color: ";
    value_to_yaml(msg.fill_color, out);
    out << "\n";
  }

  // member: hidden
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hidden: ";
    value_to_yaml(msg.hidden, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MapShape & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MapShape>()
{
  return "hqv_public_interface::msg::MapShape";
}

template<>
inline const char * name<hqv_public_interface::msg::MapShape>()
{
  return "hqv_public_interface/msg/MapShape";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MapShape>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Polygon>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MapShape>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Polygon>::value> {};

template<>
struct is_message<hqv_public_interface::msg::MapShape>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__TRAITS_HPP_
