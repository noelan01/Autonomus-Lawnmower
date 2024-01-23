// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerCoDriverResult.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ICODRIVER_RESULT_DISABLED'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_RESULT_DISABLED = 0
};

/// Constant 'ICODRIVER_RESULT_COMMAND_COMPLETED'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_RESULT_COMMAND_COMPLETED = 1
};

/// Constant 'ICODRIVER_RESULT_COMMAND_INCOMPLETE'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_RESULT_COMMAND_INCOMPLETE = 2
};

/// Constant 'ICODRIVER_RESULT_COMMAND_PENDING'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_RESULT_COMMAND_PENDING = 3
};

/// Constant 'ICODRIVER_RESULT_COMMAND_TIMEOUT'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_RESULT_COMMAND_TIMEOUT = 4
};

/// Constant 'ICODRIVER_RESULT_COMMAND_ACCEPTED'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_RESULT_COMMAND_ACCEPTED = 5
};

/// Constant 'ICODRIVER_RESULT_COMMAND_REJECTED'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_RESULT_COMMAND_REJECTED = 6
};

/// Constant 'ICODRIVER_RESULT_NO_ACTIVE_COMMAND'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_RESULT_NO_ACTIVE_COMMAND = 7
};

/// Constant 'ICODRIVER_COMMAND_NONE'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_COMMAND_NONE = 0
};

/// Constant 'ICODRIVER_COMMAND_ABORT'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_COMMAND_ABORT = 1
};

/// Constant 'ICODRIVER_COMMAND_ROTATE'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_COMMAND_ROTATE = 2
};

/// Constant 'ICODRIVER_COMMAND_TURN'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_COMMAND_TURN = 3
};

/// Constant 'ICODRIVER_COMMAND_MOVE_TO'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_COMMAND_MOVE_TO = 4
};

/// Constant 'ICODRIVER_COMMAND_REDUCE_SPEED'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_COMMAND_REDUCE_SPEED = 5
};

/// Constant 'ICODRIVER_COMMAND_FAILED'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_COMMAND_FAILED = 0
};

/// Constant 'ICODRIVER_COMMAND_PASSED'.
enum
{
  hqv_public_interface__msg__MowerCoDriverResult__ICODRIVER_COMMAND_PASSED = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/MowerCoDriverResult in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerCoDriverResult
{
  std_msgs__msg__Header header;
  uint8_t result;
  uint8_t command;
  uint8_t resultcode;
} hqv_public_interface__msg__MowerCoDriverResult;

// Struct for a sequence of hqv_public_interface__msg__MowerCoDriverResult.
typedef struct hqv_public_interface__msg__MowerCoDriverResult__Sequence
{
  hqv_public_interface__msg__MowerCoDriverResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerCoDriverResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_RESULT__STRUCT_H_
