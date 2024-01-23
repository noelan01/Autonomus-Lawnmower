// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerSystemPower.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ISYSTEMPOWER_MODE_POWER_UP_AND_INIT'.
enum
{
  hqv_public_interface__msg__MowerSystemPower__ISYSTEMPOWER_MODE_POWER_UP_AND_INIT = 0
};

/// Constant 'ISYSTEMPOWER_MODE_ACTIVE'.
enum
{
  hqv_public_interface__msg__MowerSystemPower__ISYSTEMPOWER_MODE_ACTIVE = 1
};

/// Constant 'ISYSTEMPOWER_MODE_STANDBY'.
enum
{
  hqv_public_interface__msg__MowerSystemPower__ISYSTEMPOWER_MODE_STANDBY = 2
};

/// Constant 'ISYSTEMPOWER_MODE_SHUTDOWN'.
enum
{
  hqv_public_interface__msg__MowerSystemPower__ISYSTEMPOWER_MODE_SHUTDOWN = 3
};

/// Constant 'ISYSTEMPOWER_MODE_SWITCH_TO_STANDBY'.
enum
{
  hqv_public_interface__msg__MowerSystemPower__ISYSTEMPOWER_MODE_SWITCH_TO_STANDBY = 4
};

/// Constant 'ISYSTEMPOWER_MODE_SWITCH_TO_SHUTDOWN'.
enum
{
  hqv_public_interface__msg__MowerSystemPower__ISYSTEMPOWER_MODE_SWITCH_TO_SHUTDOWN = 5
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/MowerSystemPower in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerSystemPower
{
  std_msgs__msg__Header header;
  uint8_t mode;
  uint32_t countdown;
} hqv_public_interface__msg__MowerSystemPower;

// Struct for a sequence of hqv_public_interface__msg__MowerSystemPower.
typedef struct hqv_public_interface__msg__MowerSystemPower__Sequence
{
  hqv_public_interface__msg__MowerSystemPower * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerSystemPower__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SYSTEM_POWER__STRUCT_H_
