// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/GnssRtkCollection.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/GnssRtkCollection in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__GnssRtkCollection
{
  std_msgs__msg__Header header;
  uint64_t time_ms;
  uint16_t ref_station_id;
  uint32_t i_tow;
  int32_t rel_pos_n;
  int32_t rel_pos_e;
  int32_t rel_pos_d;
  int8_t rel_pos_hpn;
  int8_t rel_pos_hpe;
  int8_t rel_pos_hpd;
  uint32_t acc_n;
  uint32_t acc_e;
  uint32_t acc_d;
  uint32_t flags;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t validity_flags;
  uint32_t t_acc;
  uint8_t fix_type;
  uint8_t fix_status_flags;
  uint8_t flags2;
  uint8_t num_sv;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t h_msl;
  uint32_t h_acc;
  uint32_t v_acc;
  int32_t vel_n;
  int32_t vel_e;
  int32_t vel_d;
  int32_t g_speed;
  int32_t head_mot;
  uint32_t s_acc;
  uint32_t head_acc;
} hqv_public_interface__msg__GnssRtkCollection;

// Struct for a sequence of hqv_public_interface__msg__GnssRtkCollection.
typedef struct hqv_public_interface__msg__GnssRtkCollection__Sequence
{
  hqv_public_interface__msg__GnssRtkCollection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__GnssRtkCollection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__GNSS_RTK_COLLECTION__STRUCT_H_
