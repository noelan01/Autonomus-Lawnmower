// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:msg/GnssRtkCollection.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__TRAITS_HPP_

#include "hqv_public_interface/msg/detail/gnss_rtk_collection__struct.hpp"
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
  const hqv_public_interface::msg::GnssRtkCollection & msg,
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

  // member: flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flags: ";
    value_to_yaml(msg.flags, out);
    out << "\n";
  }

  // member: year
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "year: ";
    value_to_yaml(msg.year, out);
    out << "\n";
  }

  // member: month
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "month: ";
    value_to_yaml(msg.month, out);
    out << "\n";
  }

  // member: day
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "day: ";
    value_to_yaml(msg.day, out);
    out << "\n";
  }

  // member: hour
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hour: ";
    value_to_yaml(msg.hour, out);
    out << "\n";
  }

  // member: min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min: ";
    value_to_yaml(msg.min, out);
    out << "\n";
  }

  // member: sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sec: ";
    value_to_yaml(msg.sec, out);
    out << "\n";
  }

  // member: validity_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "validity_flags: ";
    value_to_yaml(msg.validity_flags, out);
    out << "\n";
  }

  // member: t_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_acc: ";
    value_to_yaml(msg.t_acc, out);
    out << "\n";
  }

  // member: fix_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fix_type: ";
    value_to_yaml(msg.fix_type, out);
    out << "\n";
  }

  // member: fix_status_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fix_status_flags: ";
    value_to_yaml(msg.fix_status_flags, out);
    out << "\n";
  }

  // member: flags2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flags2: ";
    value_to_yaml(msg.flags2, out);
    out << "\n";
  }

  // member: num_sv
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_sv: ";
    value_to_yaml(msg.num_sv, out);
    out << "\n";
  }

  // member: lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon: ";
    value_to_yaml(msg.lon, out);
    out << "\n";
  }

  // member: lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat: ";
    value_to_yaml(msg.lat, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: h_msl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "h_msl: ";
    value_to_yaml(msg.h_msl, out);
    out << "\n";
  }

  // member: h_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "h_acc: ";
    value_to_yaml(msg.h_acc, out);
    out << "\n";
  }

  // member: v_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_acc: ";
    value_to_yaml(msg.v_acc, out);
    out << "\n";
  }

  // member: vel_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_n: ";
    value_to_yaml(msg.vel_n, out);
    out << "\n";
  }

  // member: vel_e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_e: ";
    value_to_yaml(msg.vel_e, out);
    out << "\n";
  }

  // member: vel_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_d: ";
    value_to_yaml(msg.vel_d, out);
    out << "\n";
  }

  // member: g_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g_speed: ";
    value_to_yaml(msg.g_speed, out);
    out << "\n";
  }

  // member: head_mot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "head_mot: ";
    value_to_yaml(msg.head_mot, out);
    out << "\n";
  }

  // member: s_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_acc: ";
    value_to_yaml(msg.s_acc, out);
    out << "\n";
  }

  // member: head_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "head_acc: ";
    value_to_yaml(msg.head_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::msg::GnssRtkCollection & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::msg::GnssRtkCollection>()
{
  return "hqv_public_interface::msg::GnssRtkCollection";
}

template<>
inline const char * name<hqv_public_interface::msg::GnssRtkCollection>()
{
  return "hqv_public_interface/msg/GnssRtkCollection";
}

template<>
struct has_fixed_size<hqv_public_interface::msg::GnssRtkCollection>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::msg::GnssRtkCollection>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hqv_public_interface::msg::GnssRtkCollection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__TRAITS_HPP_
