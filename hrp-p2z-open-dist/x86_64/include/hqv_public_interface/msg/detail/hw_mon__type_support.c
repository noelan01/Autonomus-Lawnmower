// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/HwMon.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/hw_mon__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/hw_mon__functions.h"
#include "hqv_public_interface/msg/detail/hw_mon__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void HwMon__rosidl_typesupport_introspection_c__HwMon_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__HwMon__init(message_memory);
}

void HwMon__rosidl_typesupport_introspection_c__HwMon_fini_function(void * message_memory)
{
  hqv_public_interface__msg__HwMon__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember HwMon__rosidl_typesupport_introspection_c__HwMon_message_member_array[3] = {
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__HwMon, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cpu_freq",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__HwMon, cpu_freq),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "used_memory",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__HwMon, used_memory),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers HwMon__rosidl_typesupport_introspection_c__HwMon_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "HwMon",  // message name
  3,  // number of fields
  sizeof(hqv_public_interface__msg__HwMon),
  HwMon__rosidl_typesupport_introspection_c__HwMon_message_member_array,  // message members
  HwMon__rosidl_typesupport_introspection_c__HwMon_init_function,  // function to initialize message memory (memory has to be allocated)
  HwMon__rosidl_typesupport_introspection_c__HwMon_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t HwMon__rosidl_typesupport_introspection_c__HwMon_message_type_support_handle = {
  0,
  &HwMon__rosidl_typesupport_introspection_c__HwMon_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, HwMon)() {
  if (!HwMon__rosidl_typesupport_introspection_c__HwMon_message_type_support_handle.typesupport_identifier) {
    HwMon__rosidl_typesupport_introspection_c__HwMon_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &HwMon__rosidl_typesupport_introspection_c__HwMon_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
