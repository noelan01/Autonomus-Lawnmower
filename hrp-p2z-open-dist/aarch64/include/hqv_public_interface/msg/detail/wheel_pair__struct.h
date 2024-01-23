// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/WheelPair.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/WheelPair in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__WheelPair
{
  std_msgs__msg__Header header;
  uint16_t power_left;
  uint16_t power_right;
  int32_t counter_left;
  int32_t counter_right;
  uint8_t free_left;
  uint8_t free_right;
  float deltadistance_left;
  float deltadistance_right;
  uint8_t state_left;
  uint8_t state_right;
  int16_t current_left;
  int16_t current_right;
  float speed_left;
  float speed_right;
  bool isrunning_left;
  bool isrunning_right;
} hqv_public_interface__msg__WheelPair;

// Struct for a sequence of hqv_public_interface__msg__WheelPair.
typedef struct hqv_public_interface__msg__WheelPair__Sequence
{
  hqv_public_interface__msg__WheelPair * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__WheelPair__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__WHEEL_PAIR__STRUCT_H_
