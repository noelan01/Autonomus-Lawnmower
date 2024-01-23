// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MowerMowerMode.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/mower_mower_mode__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/mower_mower_mode__functions.h"
#include "hqv_public_interface/msg/detail/mower_mower_mode__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `description`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MowerMowerMode__init(message_memory);
}

void MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MowerMowerMode__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerMowerMode, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerMowerMode, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerMowerMode, description),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MowerMowerMode",  // message name
  3,  // number of fields
  sizeof(hqv_public_interface__msg__MowerMowerMode),
  MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_member_array,  // message members
  MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_type_support_handle = {
  0,
  &MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MowerMowerMode)() {
  MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_type_support_handle.typesupport_identifier) {
    MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerMowerMode__rosidl_typesupport_introspection_c__MowerMowerMode_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
