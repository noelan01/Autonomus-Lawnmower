// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerGnssTime.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__STRUCT_H_

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

// Struct defined in msg/MowerGnssTime in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerGnssTime
{
  std_msgs__msg__Header header;
  uint32_t itow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  int32_t nano;
  uint32_t timeaccuracy;
  bool timevalid;
  bool datevalid;
} hqv_public_interface__msg__MowerGnssTime;

// Struct for a sequence of hqv_public_interface__msg__MowerGnssTime.
typedef struct hqv_public_interface__msg__MowerGnssTime__Sequence
{
  hqv_public_interface__msg__MowerGnssTime * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerGnssTime__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_TIME__STRUCT_H_
