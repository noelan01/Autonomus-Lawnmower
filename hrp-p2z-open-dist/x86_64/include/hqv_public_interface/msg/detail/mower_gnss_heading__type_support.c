// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MowerGnssHeading.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/mower_gnss_heading__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/mower_gnss_heading__functions.h"
#include "hqv_public_interface/msg/detail/mower_gnss_heading__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MowerGnssHeading__init(message_memory);
}

void MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MowerGnssHeading__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssHeading, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motion",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssHeading, motion),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mower",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssHeading, mower),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssHeading, acc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MowerGnssHeading",  // message name
  4,  // number of fields
  sizeof(hqv_public_interface__msg__MowerGnssHeading),
  MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_member_array,  // message members
  MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_type_support_handle = {
  0,
  &MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MowerGnssHeading)() {
  MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_type_support_handle.typesupport_identifier) {
    MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerGnssHeading__rosidl_typesupport_introspection_c__MowerGnssHeading_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
