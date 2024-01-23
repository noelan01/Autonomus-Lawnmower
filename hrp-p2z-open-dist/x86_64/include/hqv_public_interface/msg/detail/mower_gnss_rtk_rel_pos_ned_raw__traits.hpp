// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelPosNEDRaw.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/mower_gnss_rtk_rel_pos_ned_raw__struct.hpp"
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
  const hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg,
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

  // member: time_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_ms: ";
    value_to_yaml(msg.time_ms, out);
    out << "\n";
  }

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: reserved0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved0: ";
    value_to_yaml(msg.reserved0, out);
    out << "\n";
  }

  // member: ref_station_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ref_station_id: ";
    value_to_yaml(msg.ref_station_id, out);
    out << "\n";
  }

  // member: i_tow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i_tow: ";
    value_to_yaml(msg.i_tow, out);
    out << "\n";
  }

  // member: rel_pos_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_n: ";
    value_to_yaml(msg.rel_pos_n, out);
    out << "\n";
  }

  // member: rel_pos_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_e: ";
    value_to_yaml(msg.rel_pos_e, out);
    out << "\n";
  }

  // member: rel_pos_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_d: ";
    value_to_yaml(msg.rel_pos_d, out);
    out << "\n";
  }

  // member: rel_pos_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_length: ";
    value_to_yaml(msg.rel_pos_length, out);
    out << "\n";
  }

  // member: rel_pos_heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_heading: ";
    value_to_yaml(msg.rel_pos_heading, out);
    out << "\n";
  }

  // member: reserved1_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved1_0: ";
    value_to_yaml(msg.reserved1_0, out);
    out << "\n";
  }

  // member: reserved1_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved1_1: ";
    value_to_yaml(msg.reserved1_1, out);
    out << "\n";
  }

  // member: reserved1_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved1_2: ";
    value_to_yaml(msg.reserved1_2, out);
    out << "\n";
  }

  // member: reserved1_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved1_3: ";
    value_to_yaml(msg.reserved1_3, out);
    out << "\n";
  }

  // member: rel_pos_hpn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_hpn: ";
    value_to_yaml(msg.rel_pos_hpn, out);
    out << "\n";
  }

  // member: rel_pos_hpe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_hpe: ";
    value_to_yaml(msg.rel_pos_hpe, out);
    out << "\n";
  }

  // member: rel_pos_hpd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_hpd: ";
    value_to_yaml(msg.rel_pos_hpd, out);
    out << "\n";
  }

  // member: rel_pos_hp_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rel_pos_hp_length: ";
    value_to_yaml(msg.rel_pos_hp_length, out);
    out << "\n";
  }

  // member: acc_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_n: ";
    value_to_yaml(msg.acc_n, out);
    out << "\n";
  }

  // member: acc_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_e: ";
    value_to_yaml(msg.acc_e, out);
    out << "\n";
  }

  // member: acc_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_d: ";
    value_to_yaml(msg.acc_d, out);
    out << "\n";
  }

  // member: acc_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_length: ";
    value_to_yaml(msg.acc_length, out);
    out << "\n";
  }

  // member: acc_heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc_heading: ";
    value_to_yaml(msg.acc_heading, out);
    out << "\n";
  }

  // member: reserved2_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved2_0: ";
    value_to_yaml(msg.reserved2_0, out);
    out << "\n";
  }

  // member: reserved2_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved2_1: ";
    value_to_yaml(msg.reserved2_1, out);
    out << "\n";
  }

  // member: reserved2_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved2_2: ";
    value_to_yaml(msg.reserved2_2, out);
    out << "\n";
  }

  // member: reserved2_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reserved2_3: ";
    value_to_yaml(msg.reserved2_3, out);
    out << "\n";
  }

  // member: flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flags: ";
    value_to_yaml(msg.flags, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw>()
{
  return "hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw";
}

template<>
inline const char * name<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw>()
{
  return "hqv_public_interface/msg/MowerGnssRtkRelPosNEDRaw";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hqv_public_interface::msg::MowerGnssRtkRelPosNEDRaw>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__TRAITS_HPP_
