// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerGnssPvtRaw.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__STRUCT_H_

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

// Struct defined in msg/MowerGnssPvtRaw in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerGnssPvtRaw
{
  std_msgs__msg__Header header;
  uint64_t time_ms;
  uint32_t i_tow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t validity_flags;
  uint32_t t_acc;
  int32_t nano;
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
  uint16_t p_dop;
  uint8_t flags3;
  uint8_t reserved0_0;
  uint8_t reserved0_1;
  uint8_t reserved0_2;
  uint8_t reserved0_3;
  uint8_t reserved0_4;
  int32_t head_veh;
  int16_t mag_dec;
  uint16_t mag_acc;
} hqv_public_interface__msg__MowerGnssPvtRaw;

// Struct for a sequence of hqv_public_interface__msg__MowerGnssPvtRaw.
typedef struct hqv_public_interface__msg__MowerGnssPvtRaw__Sequence
{
  hqv_public_interface__msg__MowerGnssPvtRaw * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerGnssPvtRaw__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_PVT_RAW__STRUCT_H_
