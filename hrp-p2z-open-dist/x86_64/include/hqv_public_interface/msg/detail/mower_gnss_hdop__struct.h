// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerGnssHdop.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HDOP__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HDOP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IGNSS_SIGNALSTRENGTH_0'.
enum
{
  hqv_public_interface__msg__MowerGnssHdop__IGNSS_SIGNALSTRENGTH_0 = 0
};

/// Constant 'IGNSS_SIGNALSTRENGTH_7'.
enum
{
  hqv_public_interface__msg__MowerGnssHdop__IGNSS_SIGNALSTRENGTH_7 = 2
};

/// Constant 'IGNSS_SIGNALSTRENGTH_6'.
enum
{
  hqv_public_interface__msg__MowerGnssHdop__IGNSS_SIGNALSTRENGTH_6 = 3
};

/// Constant 'IGNSS_SIGNALSTRENGTH_5'.
enum
{
  hqv_public_interface__msg__MowerGnssHdop__IGNSS_SIGNALSTRENGTH_5 = 4
};

/// Constant 'IGNSS_SIGNALSTRENGTH_4'.
enum
{
  hqv_public_interface__msg__MowerGnssHdop__IGNSS_SIGNALSTRENGTH_4 = 5
};

/// Constant 'IGNSS_SIGNALSTRENGTH_3'.
enum
{
  hqv_public_interface__msg__MowerGnssHdop__IGNSS_SIGNALSTRENGTH_3 = 6
};

/// Constant 'IGNSS_SIGNALSTRENGTH_2'.
enum
{
  hqv_public_interface__msg__MowerGnssHdop__IGNSS_SIGNALSTRENGTH_2 = 7
};

/// Constant 'IGNSS_SIGNALSTRENGTH_1'.
enum
{
  hqv_public_interface__msg__MowerGnssHdop__IGNSS_SIGNALSTRENGTH_1 = 8
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/MowerGnssHdop in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerGnssHdop
{
  std_msgs__msg__Header header;
  uint8_t strength;
} hqv_public_interface__msg__MowerGnssHdop;

// Struct for a sequence of hqv_public_interface__msg__MowerGnssHdop.
typedef struct hqv_public_interface__msg__MowerGnssHdop__Sequence
{
  hqv_public_interface__msg__MowerGnssHdop * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerGnssHdop__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_HDOP__STRUCT_H_
