// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelPosNEDRaw.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__STRUCT_H_

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

// Struct defined in msg/MowerGnssRtkRelPosNEDRaw in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw
{
  std_msgs__msg__Header header;
  uint64_t time_ms;
  uint8_t version;
  uint8_t reserved0;
  uint16_t ref_station_id;
  uint32_t i_tow;
  int32_t rel_pos_n;
  int32_t rel_pos_e;
  int32_t rel_pos_d;
  int32_t rel_pos_length;
  int32_t rel_pos_heading;
  uint8_t reserved1_0;
  uint8_t reserved1_1;
  uint8_t reserved1_2;
  uint8_t reserved1_3;
  int8_t rel_pos_hpn;
  int8_t rel_pos_hpe;
  int8_t rel_pos_hpd;
  int8_t rel_pos_hp_length;
  uint32_t acc_n;
  uint32_t acc_e;
  uint32_t acc_d;
  uint32_t acc_length;
  uint32_t acc_heading;
  uint8_t reserved2_0;
  uint8_t reserved2_1;
  uint8_t reserved2_2;
  uint8_t reserved2_3;
  uint32_t flags;
} hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw;

// Struct for a sequence of hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw.
typedef struct hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence
{
  hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_RTK_REL_POS_NED_RAW__STRUCT_H_
