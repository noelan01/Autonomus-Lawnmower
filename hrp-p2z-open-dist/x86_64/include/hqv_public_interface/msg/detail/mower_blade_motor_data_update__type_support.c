// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MowerBladeMotorDataUpdate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/mower_blade_motor_data_update__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/mower_blade_motor_data_update__functions.h"
#include "hqv_public_interface/msg/detail/mower_blade_motor_data_update__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MowerBladeMotorDataUpdate__init(message_memory);
}

void MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MowerBladeMotorDataUpdate__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerBladeMotorDataUpdate, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerBladeMotorDataUpdate, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "direction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerBladeMotorDataUpdate, direction),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rpm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerBladeMotorDataUpdate, rpm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerBladeMotorDataUpdate, current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MowerBladeMotorDataUpdate",  // message name
  5,  // number of fields
  sizeof(hqv_public_interface__msg__MowerBladeMotorDataUpdate),
  MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_member_array,  // message members
  MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_type_support_handle = {
  0,
  &MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MowerBladeMotorDataUpdate)() {
  MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_type_support_handle.typesupport_identifier) {
    MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerBladeMotorDataUpdate__rosidl_typesupport_introspection_c__MowerBladeMotorDataUpdate_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
