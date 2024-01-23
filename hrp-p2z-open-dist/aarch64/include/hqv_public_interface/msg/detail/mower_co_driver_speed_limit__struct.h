// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerCoDriverSpeedLimit.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_SPEED_LIMIT__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_SPEED_LIMIT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/MowerCoDriverSpeedLimit in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerCoDriverSpeedLimit
{
  float speedlimit;
} hqv_public_interface__msg__MowerCoDriverSpeedLimit;

// Struct for a sequence of hqv_public_interface__msg__MowerCoDriverSpeedLimit.
typedef struct hqv_public_interface__msg__MowerCoDriverSpeedLimit__Sequence
{
  hqv_public_interface__msg__MowerCoDriverSpeedLimit * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerCoDriverSpeedLimit__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_CO_DRIVER_SPEED_LIMIT__STRUCT_H_
