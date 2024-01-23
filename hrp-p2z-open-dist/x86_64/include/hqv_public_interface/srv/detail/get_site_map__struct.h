// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:srv/GetSiteMap.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAP_SITENAME'.
enum
{
  hqv_public_interface__srv__GetSiteMap_Request__MAP_SITENAME = 0
};

/// Constant 'MAP_DATA'.
enum
{
  hqv_public_interface__srv__GetSiteMap_Request__MAP_DATA = 1
};

/// Constant 'MAP_UPDATE'.
enum
{
  hqv_public_interface__srv__GetSiteMap_Request__MAP_UPDATE = 2
};

// Struct defined in srv/GetSiteMap in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__GetSiteMap_Request
{
  uint8_t request;
} hqv_public_interface__srv__GetSiteMap_Request;

// Struct for a sequence of hqv_public_interface__srv__GetSiteMap_Request.
typedef struct hqv_public_interface__srv__GetSiteMap_Request__Sequence
{
  hqv_public_interface__srv__GetSiteMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__GetSiteMap_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'sitename'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in srv/GetSiteMap in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__GetSiteMap_Response
{
  bool loaded;
  bool valid;
  rosidl_runtime_c__String sitename;
  rosidl_runtime_c__octet__Sequence data;
} hqv_public_interface__srv__GetSiteMap_Response;

// Struct for a sequence of hqv_public_interface__srv__GetSiteMap_Response.
typedef struct hqv_public_interface__srv__GetSiteMap_Response__Sequence
{
  hqv_public_interface__srv__GetSiteMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__GetSiteMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__GET_SITE_MAP__STRUCT_H_
