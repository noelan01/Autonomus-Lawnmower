// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerGnssPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POSITION__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POSITION__STRUCT_H_

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

// Struct defined in msg/MowerGnssPosition in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerGnssPosition
{
  std_msgs__msg__Header header;
  double latitude;
  double longitude;
} hqv_public_interface__msg__MowerGnssPosition;

// Struct for a sequence of hqv_public_interface__msg__MowerGnssPosition.
typedef struct hqv_public_interface__msg__MowerGnssPosition__Sequence
{
  hqv_public_interface__msg__MowerGnssPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerGnssPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_POSITION__STRUCT_H_
