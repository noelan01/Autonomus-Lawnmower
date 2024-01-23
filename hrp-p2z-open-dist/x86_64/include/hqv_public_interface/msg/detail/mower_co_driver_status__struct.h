// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerCoDriverStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_STATUS__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ICODRIVER_STATUS_CLOSED'.
enum
{
  hqv_public_interface__msg__MowerCoDriverStatus__ICODRIVER_STATUS_CLOSED = 0
};

/// Constant 'ICODRIVER_STATUS_OPEN'.
enum
{
  hqv_public_interface__msg__MowerCoDriverStatus__ICODRIVER_STATUS_OPEN = 1
};

/// Constant 'ICODRIVER_STATUS_UNKNOWN'.
enum
{
  hqv_public_interface__msg__MowerCoDriverStatus__ICODRIVER_STATUS_UNKNOWN = 2
};

// Struct defined in msg/MowerCoDriverStatus in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerCoDriverStatus
{
  uint8_t status;
} hqv_public_interface__msg__MowerCoDriverStatus;

// Struct for a sequence of hqv_public_interface__msg__MowerCoDriverStatus.
typedef struct hqv_public_interface__msg__MowerCoDriverStatus__Sequence
{
  hqv_public_interface__msg__MowerCoDriverStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerCoDriverStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_STATUS__STRUCT_H_
