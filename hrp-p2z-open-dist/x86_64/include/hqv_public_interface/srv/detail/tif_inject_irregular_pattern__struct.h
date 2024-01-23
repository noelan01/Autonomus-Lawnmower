// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:srv/TifInjectIrregularPattern.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/TifInjectIrregularPattern in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__TifInjectIrregularPattern_Request
{
  uint8_t set_state;
} hqv_public_interface__srv__TifInjectIrregularPattern_Request;

// Struct for a sequence of hqv_public_interface__srv__TifInjectIrregularPattern_Request.
typedef struct hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence
{
  hqv_public_interface__srv__TifInjectIrregularPattern_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/TifInjectIrregularPattern in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__TifInjectIrregularPattern_Response
{
  uint8_t status;
} hqv_public_interface__srv__TifInjectIrregularPattern_Response;

// Struct for a sequence of hqv_public_interface__srv__TifInjectIrregularPattern_Response.
typedef struct hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence
{
  hqv_public_interface__srv__TifInjectIrregularPattern_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__TIF_INJECT_IRREGULAR_PATTERN__STRUCT_H_
