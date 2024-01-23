// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerSlamHandling.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SLAM_HANDLE_START'.
enum
{
  hqv_public_interface__msg__MowerSlamHandling__SLAM_HANDLE_START = 0
};

/// Constant 'SLAM_HANDLE_STOP'.
enum
{
  hqv_public_interface__msg__MowerSlamHandling__SLAM_HANDLE_STOP = 1
};

/// Constant 'SLAM_HANDLE_RESET'.
enum
{
  hqv_public_interface__msg__MowerSlamHandling__SLAM_HANDLE_RESET = 2
};

// Struct defined in msg/MowerSlamHandling in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerSlamHandling
{
  uint8_t slam_handle;
} hqv_public_interface__msg__MowerSlamHandling;

// Struct for a sequence of hqv_public_interface__msg__MowerSlamHandling.
typedef struct hqv_public_interface__msg__MowerSlamHandling__Sequence
{
  hqv_public_interface__msg__MowerSlamHandling * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerSlamHandling__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_HANDLING__STRUCT_H_
