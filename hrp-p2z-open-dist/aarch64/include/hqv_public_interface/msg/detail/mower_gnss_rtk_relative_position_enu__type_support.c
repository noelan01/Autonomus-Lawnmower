// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelativePositionENU.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/mower_gnss_rtk_relative_position_enu__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/mower_gnss_rtk_relative_position_enu__functions.h"
#include "hqv_public_interface/msg/detail/mower_gnss_rtk_relative_position_enu__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MowerGnssRtkRelativePositionENU__init(message_memory);
}

void MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MowerGnssRtkRelativePositionENU__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkRelativePositionENU, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "east",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkRelativePositionENU, east),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "north",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkRelativePositionENU, north),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "up",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkRelativePositionENU, up),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accuracy_east",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkRelativePositionENU, accuracy_east),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accuracy_north",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkRelativePositionENU, accuracy_north),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accuracy_up",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MowerGnssRtkRelativePositionENU, accuracy_up),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MowerGnssRtkRelativePositionENU",  // message name
  7,  // number of fields
  sizeof(hqv_public_interface__msg__MowerGnssRtkRelativePositionENU),
  MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_member_array,  // message members
  MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_type_support_handle = {
  0,
  &MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MowerGnssRtkRelativePositionENU)() {
  MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_type_support_handle.typesupport_identifier) {
    MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerGnssRtkRelativePositionENU__rosidl_typesupport_introspection_c__MowerGnssRtkRelativePositionENU_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
