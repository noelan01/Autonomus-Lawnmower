// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:srv/GetCurrentSubMap.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_CURRENT_SUB_MAP__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_CURRENT_SUB_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/GetCurrentSubMap in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__GetCurrentSubMap_Request
{
  uint8_t structure_needs_at_least_one_member;
} hqv_public_interface__srv__GetCurrentSubMap_Request;

// Struct for a sequence of hqv_public_interface__srv__GetCurrentSubMap_Request.
typedef struct hqv_public_interface__srv__GetCurrentSubMap_Request__Sequence
{
  hqv_public_interface__srv__GetCurrentSubMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__GetCurrentSubMap_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'description'
#include "rosidl_runtime_c/string.h"
// Member 'map_info'
#include "hqv_public_interface/msg/detail/map_info__struct.h"

// Struct defined in srv/GetCurrentSubMap in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__GetCurrentSubMap_Response
{
  int64_t result;
  uint32_t mission;
  rosidl_runtime_c__String description;
  hqv_public_interface__msg__MapInfo map_info;
} hqv_public_interface__srv__GetCurrentSubMap_Response;

// Struct for a sequence of hqv_public_interface__srv__GetCurrentSubMap_Response.
typedef struct hqv_public_interface__srv__GetCurrentSubMap_Response__Sequence
{
  hqv_public_interface__srv__GetCurrentSubMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__GetCurrentSubMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_CURRENT_SUB_MAP__STRUCT_H_
