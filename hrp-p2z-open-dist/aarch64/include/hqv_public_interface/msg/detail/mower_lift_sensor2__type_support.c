// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MowerLiftSensor2.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/mower_lift_sensor2__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/mower_lift_sensor2__functions.h"
#include "hqv_public_interface/msg/detail/mower_lift_sensor2__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MowerLiftSensor2__init(message_memory);
}

void MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MowerLiftSensor2__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerLiftSensor2, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frontleft",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerLiftSensor2, frontleft),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frontright",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerLiftSensor2, frontright),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MowerLiftSensor2",  // message name
  3,  // number of fields
  sizeof(hqv_public_interface__msg__MowerLiftSensor2),
  MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_member_array,  // message members
  MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_type_support_handle = {
  0,
  &MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MowerLiftSensor2)() {
  MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_type_support_handle.typesupport_identifier) {
    MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerLiftSensor2__rosidl_typesupport_introspection_c__MowerLiftSensor2_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
