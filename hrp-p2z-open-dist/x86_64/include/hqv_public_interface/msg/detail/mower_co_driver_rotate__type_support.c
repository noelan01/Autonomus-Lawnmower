// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MowerCoDriverRotate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/mower_co_driver_rotate__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/mower_co_driver_rotate__functions.h"
#include "hqv_public_interface/msg/detail/mower_co_driver_rotate__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MowerCoDriverRotate__init(message_memory);
}

void MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MowerCoDriverRotate__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_message_member_array[1] = {
  {
    "rotateangle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerCoDriverRotate, rotateangle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MowerCoDriverRotate",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__msg__MowerCoDriverRotate),
  MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_message_member_array,  // message members
  MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_message_type_support_handle = {
  0,
  &MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MowerCoDriverRotate)() {
  if (!MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_message_type_support_handle.typesupport_identifier) {
    MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerCoDriverRotate__rosidl_typesupport_introspection_c__MowerCoDriverRotate_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
