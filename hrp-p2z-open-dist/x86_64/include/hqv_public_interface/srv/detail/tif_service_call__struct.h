// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:srv/TifServiceCall.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'nodename'
#include "rosidl_runtime_c/string.h"
// Member 'payload'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in srv/TifServiceCall in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__TifServiceCall_Request
{
  rosidl_runtime_c__String nodename;
  uint16_t family;
  uint16_t command;
  rosidl_runtime_c__octet__Sequence payload;
  uint16_t timeout;
} hqv_public_interface__srv__TifServiceCall_Request;

// Struct for a sequence of hqv_public_interface__srv__TifServiceCall_Request.
typedef struct hqv_public_interface__srv__TifServiceCall_Request__Sequence
{
  hqv_public_interface__srv__TifServiceCall_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__TifServiceCall_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'data'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in srv/TifServiceCall in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__TifServiceCall_Response
{
  uint8_t result;
  rosidl_runtime_c__octet__Sequence data;
} hqv_public_interface__srv__TifServiceCall_Response;

// Struct for a sequence of hqv_public_interface__srv__TifServiceCall_Response.
typedef struct hqv_public_interface__srv__TifServiceCall_Response__Sequence
{
  hqv_public_interface__srv__TifServiceCall_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__TifServiceCall_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_SERVICE_CALL__STRUCT_H_
