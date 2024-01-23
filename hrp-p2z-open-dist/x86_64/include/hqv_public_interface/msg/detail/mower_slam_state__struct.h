// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerSlamState.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_STATE__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_RUNNING'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_RUNNING = 0
};

/// Constant 'STATE_IDLE'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_IDLE = 1
};

/// Constant 'STATE_STARTED'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_STARTED = 2
};

/// Constant 'STATE_CONNECTED'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_CONNECTED = 3
};

/// Constant 'STATE_READY'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_READY = 4
};

/// Constant 'STATE_CALIBRATING'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_CALIBRATING = 5
};

/// Constant 'STATE_FILTER_WARM_UP'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_FILTER_WARM_UP = 6
};

/// Constant 'STATE_WAITING_FOR_GLOBAL_POS'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_WAITING_FOR_GLOBAL_POS = 7
};

/// Constant 'STATE_UNKNOWN'.
enum
{
  hqv_public_interface__msg__MowerSlamState__STATE_UNKNOWN = 8
};

// Struct defined in msg/MowerSlamState in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerSlamState
{
  uint8_t state;
} hqv_public_interface__msg__MowerSlamState;

// Struct for a sequence of hqv_public_interface__msg__MowerSlamState.
typedef struct hqv_public_interface__msg__MowerSlamState__Sequence
{
  hqv_public_interface__msg__MowerSlamState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerSlamState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_STATE__STRUCT_H_
