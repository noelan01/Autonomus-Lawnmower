// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerCoDriverMoveTo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MowerCoDriverMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerCoDriverMoveTo
{
  int32_t x;
  int32_t y;
} hqv_public_interface__msg__MowerCoDriverMoveTo;

// Struct for a sequence of hqv_public_interface__msg__MowerCoDriverMoveTo.
typedef struct hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence
{
  hqv_public_interface__msg__MowerCoDriverMoveTo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_MOVE_TO__STRUCT_H_
