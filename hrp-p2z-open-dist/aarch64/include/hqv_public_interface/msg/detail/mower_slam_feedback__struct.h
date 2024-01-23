// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerSlamFeedback.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FEEDBACK_NORMAL'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_NORMAL = 0
};

/// Constant 'FEEDBACK_FULL_STOP'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_FULL_STOP = 1
};

/// Constant 'FEEDBACK_TURN_IN_PLACE'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_TURN_IN_PLACE = 2
};

/// Constant 'FEEDBACK_ADAPT_MAX_VELOCITY'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_ADAPT_MAX_VELOCITY = 3
};

/// Constant 'FEEDBACK_CHANGE_HEADING'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_CHANGE_HEADING = 4
};

/// Constant 'FEEDBACK_DRIVE_STRAIGHT'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_DRIVE_STRAIGHT = 5
};

/// Constant 'FEEDBACK_DRIVE_TO'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_DRIVE_TO = 6
};

/// Constant 'FEEDBACK_CALIBRATION_DONE'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_CALIBRATION_DONE = 7
};

/// Constant 'FEEDBACK_RTK_ANOMALY'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_RTK_ANOMALY = 8
};

/// Constant 'FEEDBACK_UNKNOWN'.
enum
{
  hqv_public_interface__msg__MowerSlamFeedback__FEEDBACK_UNKNOWN = 9
};

// Struct defined in msg/MowerSlamFeedback in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerSlamFeedback
{
  uint8_t feedback;
  int32_t feedback_data_1;
  int32_t feedback_data_2;
} hqv_public_interface__msg__MowerSlamFeedback;

// Struct for a sequence of hqv_public_interface__msg__MowerSlamFeedback.
typedef struct hqv_public_interface__msg__MowerSlamFeedback__Sequence
{
  hqv_public_interface__msg__MowerSlamFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerSlamFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SLAM_FEEDBACK__STRUCT_H_
