// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:srv/HwMonService.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/HwMonService in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__HwMonService_Request
{
  uint8_t structure_needs_at_least_one_member;
} hqv_public_interface__srv__HwMonService_Request;

// Struct for a sequence of hqv_public_interface__srv__HwMonService_Request.
typedef struct hqv_public_interface__srv__HwMonService_Request__Sequence
{
  hqv_public_interface__srv__HwMonService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__HwMonService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'hw_monitor'
#include "hqv_public_interface/msg/detail/hw_mon__struct.h"

// Struct defined in srv/HwMonService in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__HwMonService_Response
{
  hqv_public_interface__msg__HwMon hw_monitor;
} hqv_public_interface__srv__HwMonService_Response;

// Struct for a sequence of hqv_public_interface__srv__HwMonService_Response.
typedef struct hqv_public_interface__srv__HwMonService_Response__Sequence
{
  hqv_public_interface__srv__HwMonService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__HwMonService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__HW_MON_SERVICE__STRUCT_H_
