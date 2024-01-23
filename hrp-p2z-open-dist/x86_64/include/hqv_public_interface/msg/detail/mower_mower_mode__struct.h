// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerMowerMode.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MOWERAPP_MODE_AUTO'.
enum
{
  hqv_public_interface__msg__MowerMowerMode__MOWERAPP_MODE_AUTO = 0
};

/// Constant 'MOWERAPP_MODE_MANUAL'.
enum
{
  hqv_public_interface__msg__MowerMowerMode__MOWERAPP_MODE_MANUAL = 1
};

/// Constant 'MOWERAPP_MODE_HOME'.
enum
{
  hqv_public_interface__msg__MowerMowerMode__MOWERAPP_MODE_HOME = 2
};

/// Constant 'MOWERAPP_MODE_DEMO'.
enum
{
  hqv_public_interface__msg__MowerMowerMode__MOWERAPP_MODE_DEMO = 3
};

/// Constant 'MOWERAPP_MODE_POI'.
enum
{
  hqv_public_interface__msg__MowerMowerMode__MOWERAPP_MODE_POI = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'description'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/MowerMowerMode in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerMowerMode
{
  std_msgs__msg__Header header;
  uint8_t mode;
  rosidl_runtime_c__String description;
} hqv_public_interface__msg__MowerMowerMode;

// Struct for a sequence of hqv_public_interface__msg__MowerMowerMode.
typedef struct hqv_public_interface__msg__MowerMowerMode__Sequence
{
  hqv_public_interface__msg__MowerMowerMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerMowerMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_MODE__STRUCT_H_
