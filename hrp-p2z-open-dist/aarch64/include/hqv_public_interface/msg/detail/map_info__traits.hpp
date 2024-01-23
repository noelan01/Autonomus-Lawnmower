// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MapInfo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/map_info__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'cs_pos'
// Member 'pc_pos'
#include "geometry_msgs/msg/detail/point32__traits.hpp"
// Member 'charging_station'
// Member 'working_areas'
// Member 'transport_areas'
// Member 'forbidden_areas'
// Member 'hidden_areas'
// Member 'transport_paths'
// Member 'poi_paths'
#include "hqv_public_interface/msg/detail/map_shape__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::msg::MapInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_name: ";
    value_to_yaml(msg.map_name, out);
    out << "\n";
  }

  // member: cs_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cs_pos:\n";
    to_yaml(msg.cs_pos, out, indentation + 2);
  }

  // member: pc_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pc_pos:\n";
    to_yaml(msg.pc_pos, out, indentation + 2);
  }

  // member: charging_station
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charging_station:\n";
    to_yaml(msg.charging_station, out, indentation + 2);
  }

  // member: working_areas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.working_areas.size() == 0) {
      out << "working_areas: []\n";
    } else {
      out << "working_areas:\n";
      for (auto item : msg.working_areas) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: transport_areas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.transport_areas.size() == 0) {
      out << "transport_areas: []\n";
    } else {
      out << "transport_areas:\n";
      for (auto item : msg.transport_areas) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: forbidden_areas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.forbidden_areas.size() == 0) {
      out << "forbidden_areas: []\n";
    } else {
      out << "forbidden_areas:\n";
      for (auto item : msg.forbidden_areas) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: hidden_areas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.hidden_areas.size() == 0) {
      out << "hidden_areas: []\n";
    } else {
      out << "hidden_areas:\n";
      for (auto item : msg.hidden_areas) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: transport_paths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.transport_paths.size() == 0) {
      out << "transport_paths: []\n";
    } else {
      out << "transport_paths:\n";
      for (auto item : msg.transport_paths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: poi_paths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poi_paths.size() == 0) {
      out << "poi_paths: []\n";
    } else {
      out << "poi_paths:\n";
      for (auto item : msg.poi_paths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MapInfo & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MapInfo>()
{
  return "hqv_public_interface::msg::MapInfo";
}

template<>
inline const char * name<hqv_public_interface::msg::MapInfo>()
{
  return "hqv_public_interface/msg/MapInfo";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MapInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MapInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hqv_public_interface::msg::MapInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__TRAITS_HPP_
