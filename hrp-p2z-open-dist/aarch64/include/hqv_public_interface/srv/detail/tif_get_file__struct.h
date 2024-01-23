// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:srv/TifGetFile.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'filename'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/TifGetFile in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__TifGetFile_Request
{
  rosidl_runtime_c__String filename;
} hqv_public_interface__srv__TifGetFile_Request;

// Struct for a sequence of hqv_public_interface__srv__TifGetFile_Request.
typedef struct hqv_public_interface__srv__TifGetFile_Request__Sequence
{
  hqv_public_interface__srv__TifGetFile_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__TifGetFile_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in srv/TifGetFile in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__TifGetFile_Response
{
  uint8_t status;
  rosidl_runtime_c__octet__Sequence data;
} hqv_public_interface__srv__TifGetFile_Response;

// Struct for a sequence of hqv_public_interface__srv__TifGetFile_Response.
typedef struct hqv_public_interface__srv__TifGetFile_Response__Sequence
{
  hqv_public_interface__srv__TifGetFile_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__TifGetFile_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_GET_FILE__STRUCT_H_
