// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:srv/MowerSlamMapHandling.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MESSAGE_TYPE_GET_ACTIVE_MAP'.
enum
{
  hqv_public_interface__srv__MowerSlamMapHandling_Request__MESSAGE_TYPE_GET_ACTIVE_MAP = 1
};

/// Constant 'MESSAGE_TYPE_LIST_MAPS'.
enum
{
  hqv_public_interface__srv__MowerSlamMapHandling_Request__MESSAGE_TYPE_LIST_MAPS = 4
};

// Struct defined in srv/MowerSlamMapHandling in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__MowerSlamMapHandling_Request
{
  uint8_t message_type;
  uint32_t map_id;
} hqv_public_interface__srv__MowerSlamMapHandling_Request;

// Struct for a sequence of hqv_public_interface__srv__MowerSlamMapHandling_Request.
typedef struct hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence
{
  hqv_public_interface__srv__MowerSlamMapHandling_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__MowerSlamMapHandling_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/MowerSlamMapHandling in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__MowerSlamMapHandling_Response
{
  uint32_t map_id;
  uint32_t map_ids[10];
} hqv_public_interface__srv__MowerSlamMapHandling_Response;

// Struct for a sequence of hqv_public_interface__srv__MowerSlamMapHandling_Response.
typedef struct hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence
{
  hqv_public_interface__srv__MowerSlamMapHandling_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__MowerSlamMapHandling_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_SLAM_MAP_HANDLING__STRUCT_H_
