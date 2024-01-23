// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerGnssState.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IGNSS_STATE_STARTUP'.
enum
{
  hqv_public_interface__msg__MowerGnssState__IGNSS_STATE_STARTUP = 0
};

/// Constant 'IGNSS_STATE_OK'.
enum
{
  hqv_public_interface__msg__MowerGnssState__IGNSS_STATE_OK = 1
};

/// Constant 'IGNSS_STATE_HALTED'.
enum
{
  hqv_public_interface__msg__MowerGnssState__IGNSS_STATE_HALTED = 2
};

/// Constant 'IGNSS_STATE_ERROR'.
enum
{
  hqv_public_interface__msg__MowerGnssState__IGNSS_STATE_ERROR = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/MowerGnssState in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerGnssState
{
  std_msgs__msg__Header header;
  uint8_t state;
} hqv_public_interface__msg__MowerGnssState;

// Struct for a sequence of hqv_public_interface__msg__MowerGnssState.
typedef struct hqv_public_interface__msg__MowerGnssState__Sequence
{
  hqv_public_interface__msg__MowerGnssState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerGnssState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_GNSS_STATE__STRUCT_H_
