// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerLedStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__STRUCT_H_

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

// Struct defined in msg/MowerLedStatus in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerLedStatus
{
  uint8_t mower_spp_state;
  uint8_t mower_app_activity;
  uint8_t extended_status_host_message;
  uint8_t operation_manager_state;
  bool in_charging_station;
  bool is_stopped;
  uint32_t color;
  uint8_t led_mode;
  uint16_t period;
  uint8_t log_id;
  std_msgs__msg__Header header;
} hqv_public_interface__msg__MowerLedStatus;

// Struct for a sequence of hqv_public_interface__msg__MowerLedStatus.
typedef struct hqv_public_interface__msg__MowerLedStatus__Sequence
{
  hqv_public_interface__msg__MowerLedStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerLedStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__STRUCT_H_
