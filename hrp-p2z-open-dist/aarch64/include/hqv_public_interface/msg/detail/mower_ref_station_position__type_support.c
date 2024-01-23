// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/mower_ref_station_position__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/mower_ref_station_position__functions.h"
#include "hqv_public_interface/msg/detail/mower_ref_station_position__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MowerRefStationPosition__init(message_memory);
}

void MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MowerRefStationPosition__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_message_member_array[3] = {
  {
    "longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerRefStationPosition, longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerRefStationPosition, latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerRefStationPosition, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MowerRefStationPosition",  // message name
  3,  // number of fields
  sizeof(hqv_public_interface__msg__MowerRefStationPosition),
  MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_message_member_array,  // message members
  MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_message_type_support_handle = {
  0,
  &MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MowerRefStationPosition)() {
  if (!MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_message_type_support_handle.typesupport_identifier) {
    MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerRefStationPosition__rosidl_typesupport_introspection_c__MowerRefStationPosition_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
