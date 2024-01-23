// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MapShape.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'UNDEFIED_AREA'.
enum
{
  hqv_public_interface__msg__MapShape__UNDEFIED_AREA = -1
};

/// Constant 'WORKING_AREA'.
enum
{
  hqv_public_interface__msg__MapShape__WORKING_AREA = 0
};

/// Constant 'TRANSPORT_AREA'.
enum
{
  hqv_public_interface__msg__MapShape__TRANSPORT_AREA = 1
};

/// Constant 'FORBIDDEN_AREA'.
enum
{
  hqv_public_interface__msg__MapShape__FORBIDDEN_AREA = 2
};

/// Constant 'HIDDEN_AREA'.
enum
{
  hqv_public_interface__msg__MapShape__HIDDEN_AREA = 3
};

/// Constant 'TRANSPORT_PATH'.
enum
{
  hqv_public_interface__msg__MapShape__TRANSPORT_PATH = 4
};

/// Constant 'POINTS_OF_INTEREST_PATH'.
enum
{
  hqv_public_interface__msg__MapShape__POINTS_OF_INTEREST_PATH = 5
};

/// Constant 'CHARGING_STATION_PATH'.
enum
{
  hqv_public_interface__msg__MapShape__CHARGING_STATION_PATH = 6
};

// Include directives for member types
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__struct.h"

// Struct defined in msg/MapShape in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MapShape
{
  geometry_msgs__msg__Polygon polygon;
  uint8_t id[64];
  int8_t type;
  uint32_t stroke_color;
  uint32_t fill_color;
  bool hidden;
} hqv_public_interface__msg__MapShape;

// Struct for a sequence of hqv_public_interface__msg__MapShape.
typedef struct hqv_public_interface__msg__MapShape__Sequence
{
  hqv_public_interface__msg__MapShape * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MapShape__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_SHAPE__STRUCT_H_
