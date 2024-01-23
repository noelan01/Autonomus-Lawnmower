// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:srv/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MESSAGE_TYPE_INSTALLED_POSITION'.
enum
{
  hqv_public_interface__srv__MowerRefStationPosition_Request__MESSAGE_TYPE_INSTALLED_POSITION = 0
};

/// Constant 'MESSAGE_TYPE_CURRENT_POSITION'.
enum
{
  hqv_public_interface__srv__MowerRefStationPosition_Request__MESSAGE_TYPE_CURRENT_POSITION = 1
};

// Struct defined in srv/MowerRefStationPosition in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__MowerRefStationPosition_Request
{
  uint8_t message_type;
} hqv_public_interface__srv__MowerRefStationPosition_Request;

// Struct for a sequence of hqv_public_interface__srv__MowerRefStationPosition_Request.
typedef struct hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence
{
  hqv_public_interface__srv__MowerRefStationPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/MowerRefStationPosition in the package hqv_public_interface.
typedef struct hqv_public_interface__srv__MowerRefStationPosition_Response
{
  bool status;
  int32_t longitude;
  int32_t latitude;
  int32_t altitude;
} hqv_public_interface__srv__MowerRefStationPosition_Response;

// Struct for a sequence of hqv_public_interface__srv__MowerRefStationPosition_Response.
typedef struct hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence
{
  hqv_public_interface__srv__MowerRefStationPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__STRUCT_H_
