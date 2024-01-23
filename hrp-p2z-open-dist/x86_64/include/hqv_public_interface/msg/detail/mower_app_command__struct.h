// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerAppCommand.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_APP_COMMAND__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_APP_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'START'.
enum
{
  hqv_public_interface__msg__MowerAppCommand__START = 0
};

/// Constant 'PAUSE'.
enum
{
  hqv_public_interface__msg__MowerAppCommand__PAUSE = 1
};

/// Constant 'ABORT_START'.
enum
{
  hqv_public_interface__msg__MowerAppCommand__ABORT_START = 2
};

/// Constant 'CONFIRM_ERROR'.
enum
{
  hqv_public_interface__msg__MowerAppCommand__CONFIRM_ERROR = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/MowerAppCommand in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerAppCommand
{
  std_msgs__msg__Header header;
  uint8_t command;
} hqv_public_interface__msg__MowerAppCommand;

// Struct for a sequence of hqv_public_interface__msg__MowerAppCommand.
typedef struct hqv_public_interface__msg__MowerAppCommand__Sequence
{
  hqv_public_interface__msg__MowerAppCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerAppCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_APP_COMMAND__STRUCT_H_
