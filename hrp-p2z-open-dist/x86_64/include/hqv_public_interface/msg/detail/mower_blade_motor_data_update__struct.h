// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerBladeMotorDataUpdate.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__STRUCT_H_

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

// Struct defined in msg/MowerBladeMotorDataUpdate in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerBladeMotorDataUpdate
{
  std_msgs__msg__Header header;
  uint8_t state;
  uint8_t direction;
  uint16_t rpm;
  uint16_t current;
} hqv_public_interface__msg__MowerBladeMotorDataUpdate;

// Struct for a sequence of hqv_public_interface__msg__MowerBladeMotorDataUpdate.
typedef struct hqv_public_interface__msg__MowerBladeMotorDataUpdate__Sequence
{
  hqv_public_interface__msg__MowerBladeMotorDataUpdate * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerBladeMotorDataUpdate__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_BLADE_MOTOR_DATA_UPDATE__STRUCT_H_
