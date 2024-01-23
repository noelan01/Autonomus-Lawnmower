// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MowerGnssRtkSolution.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/mower_gnss_rtk_solution__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/mower_gnss_rtk_solution__functions.h"
#include "hqv_public_interface/msg/detail/mower_gnss_rtk_solution__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MowerGnssRtkSolution__init(message_memory);
}

void MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MowerGnssRtkSolution__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkSolution, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "solution",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkSolution, solution),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MowerGnssRtkSolution",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__msg__MowerGnssRtkSolution),
  MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_member_array,  // message members
  MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_type_support_handle = {
  0,
  &MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MowerGnssRtkSolution)() {
  MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_type_support_handle.typesupport_identifier) {
    MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerGnssRtkSolution__rosidl_typesupport_introspection_c__MowerGnssRtkSolution_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
