// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MapInfo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'map_name'
#include "rosidl_runtime_c/string.h"
// Member 'cs_pos'
// Member 'pc_pos'
#include "geometry_msgs/msg/detail/point32__struct.h"
// Member 'charging_station'
// Member 'working_areas'
// Member 'transport_areas'
// Member 'forbidden_areas'
// Member 'hidden_areas'
// Member 'transport_paths'
// Member 'poi_paths'
#include "hqv_public_interface/msg/detail/map_shape__struct.h"

// Struct defined in msg/MapInfo in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MapInfo
{
  rosidl_runtime_c__String map_name;
  geometry_msgs__msg__Point32 cs_pos;
  geometry_msgs__msg__Point32 pc_pos;
  hqv_public_interface__msg__MapShape charging_station;
  hqv_public_interface__msg__MapShape__Sequence working_areas;
  hqv_public_interface__msg__MapShape__Sequence transport_areas;
  hqv_public_interface__msg__MapShape__Sequence forbidden_areas;
  hqv_public_interface__msg__MapShape__Sequence hidden_areas;
  hqv_public_interface__msg__MapShape__Sequence transport_paths;
  hqv_public_interface__msg__MapShape__Sequence poi_paths;
} hqv_public_interface__msg__MapInfo;

// Struct for a sequence of hqv_public_interface__msg__MapInfo.
typedef struct hqv_public_interface__msg__MapInfo__Sequence
{
  hqv_public_interface__msg__MapInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MapInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MAP_INFO__STRUCT_H_
