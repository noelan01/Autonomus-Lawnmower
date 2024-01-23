// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerMowerActivity.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MOWERAPP_ACTIVITY_NONE'.
enum
{
  hqv_public_interface__msg__MowerMowerActivity__MOWERAPP_ACTIVITY_NONE = 0
};

/// Constant 'MOWERAPP_ACTIVITY_CHARGING'.
enum
{
  hqv_public_interface__msg__MowerMowerActivity__MOWERAPP_ACTIVITY_CHARGING = 1
};

/// Constant 'MOWERAPP_ACTIVITY_GOING_OUT'.
enum
{
  hqv_public_interface__msg__MowerMowerActivity__MOWERAPP_ACTIVITY_GOING_OUT = 2
};

/// Constant 'MOWERAPP_ACTIVITY_MOWING'.
enum
{
  hqv_public_interface__msg__MowerMowerActivity__MOWERAPP_ACTIVITY_MOWING = 3
};

/// Constant 'MOWERAPP_ACTIVITY_GOING_HOME'.
enum
{
  hqv_public_interface__msg__MowerMowerActivity__MOWERAPP_ACTIVITY_GOING_HOME = 4
};

/// Constant 'MOWERAPP_ACTIVITY_PARKED'.
enum
{
  hqv_public_interface__msg__MowerMowerActivity__MOWERAPP_ACTIVITY_PARKED = 5
};

/// Constant 'MOWERAPP_ACTIVITY_STOPPED_IN_GARDEN'.
enum
{
  hqv_public_interface__msg__MowerMowerActivity__MOWERAPP_ACTIVITY_STOPPED_IN_GARDEN = 6
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'description'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/MowerMowerActivity in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerMowerActivity
{
  std_msgs__msg__Header header;
  uint8_t activity;
  rosidl_runtime_c__String description;
} hqv_public_interface__msg__MowerMowerActivity;

// Struct for a sequence of hqv_public_interface__msg__MowerMowerActivity.
typedef struct hqv_public_interface__msg__MowerMowerActivity__Sequence
{
  hqv_public_interface__msg__MowerMowerActivity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerMowerActivity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_MOWER_ACTIVITY__STRUCT_H_
