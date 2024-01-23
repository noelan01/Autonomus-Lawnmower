// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/MowerSiteMapLoaded.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__STRUCT_H_

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
// Member 'sitename'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/MowerSiteMapLoaded in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__MowerSiteMapLoaded
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String sitename;
} hqv_public_interface__msg__MowerSiteMapLoaded;

// Struct for a sequence of hqv_public_interface__msg__MowerSiteMapLoaded.
typedef struct hqv_public_interface__msg__MowerSiteMapLoaded__Sequence
{
  hqv_public_interface__msg__MowerSiteMapLoaded * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__MowerSiteMapLoaded__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__STRUCT_H_
