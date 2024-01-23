// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerLiftSensor2.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT_SENSOR2__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT_SENSOR2__STRUCT_H_

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

// Struct defined in msg/MowerLiftSensor2 in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerLiftSensor2
{
  std_msgs__msg__Header header;
  bool frontleft;
  bool frontright;
} hqv_public_interface__msg__MowerLiftSensor2;

// Struct for a sequence of hqv_public_interface__msg__MowerLiftSensor2.
typedef struct hqv_public_interface__msg__MowerLiftSensor2__Sequence
{
  hqv_public_interface__msg__MowerLiftSensor2 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerLiftSensor2__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LIFT_SENSOR2__STRUCT_H_
