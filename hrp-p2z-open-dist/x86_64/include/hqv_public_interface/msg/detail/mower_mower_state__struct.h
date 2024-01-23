// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerMowerState.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MOWERAPP_STATE_OFF'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_OFF = 0
};

/// Constant 'MOWERAPP_STATE_WAIT_FOR_SAFETYPIN'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_WAIT_FOR_SAFETYPIN = 1
};

/// Constant 'MOWERAPP_STATE_STOPPED'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_STOPPED = 2
};

/// Constant 'MOWERAPP_STATE_FATAL_ERROR'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_FATAL_ERROR = 3
};

/// Constant 'MOWERAPP_STATE_PENDING_START'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_PENDING_START = 4
};

/// Constant 'MOWERAPP_STATE_PAUSED'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_PAUSED = 5
};

/// Constant 'MOWERAPP_STATE_IN_OPERATION'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_IN_OPERATION = 6
};

/// Constant 'MOWERAPP_STATE_RESTRICTED'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_RESTRICTED = 7
};

/// Constant 'MOWERAPP_STATE_ERROR'.
enum
{
  hqv_public_interface__msg__MowerMowerState__MOWERAPP_STATE_ERROR = 8
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'description'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/MowerMowerState in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerMowerState
{
  std_msgs__msg__Header header;
  uint8_t state;
  rosidl_runtime_c__String description;
} hqv_public_interface__msg__MowerMowerState;

// Struct for a sequence of hqv_public_interface__msg__MowerMowerState.
typedef struct hqv_public_interface__msg__MowerMowerState__Sequence
{
  hqv_public_interface__msg__MowerMowerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerMowerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_STATE__STRUCT_H_
