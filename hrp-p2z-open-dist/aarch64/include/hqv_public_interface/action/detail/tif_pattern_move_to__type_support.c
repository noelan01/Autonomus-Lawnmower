// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:action/TifPatternMoveTo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/action/detail/tif_pattern_move_to__functions.h"
#include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__TifPatternMoveTo_Goal__init(message_memory);
}

void TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_fini_function(void * message_memory)
{
  hqv_public_interface__action__TifPatternMoveTo_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_message_member_array[12] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "spd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, spd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "line_a_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, line_a_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "line_a_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, line_a_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "line_b_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, line_b_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "line_b_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, line_b_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, path_offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "randomize_path_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, randomize_path_offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "merge_at_end",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, merge_at_end),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "follow_path_backwards",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, follow_path_backwards),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "finish_action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Goal, finish_action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_message_members = {
  "hqv_public_interface__action",  // message namespace
  "TifPatternMoveTo_Goal",  // message name
  12,  // number of fields
  sizeof(hqv_public_interface__action__TifPatternMoveTo_Goal),
  TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_message_member_array,  // message members
  TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_message_type_support_handle = {
  0,
  &TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_Goal)() {
  if (!TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_message_type_support_handle.typesupport_identifier) {
    TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifPatternMoveTo_Goal__rosidl_typesupport_introspection_c__TifPatternMoveTo_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__TifPatternMoveTo_Result__init(message_memory);
}

void TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_fini_function(void * message_memory)
{
  hqv_public_interface__action__TifPatternMoveTo_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_message_member_array[1] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Result, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_message_members = {
  "hqv_public_interface__action",  // message namespace
  "TifPatternMoveTo_Result",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__action__TifPatternMoveTo_Result),
  TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_message_member_array,  // message members
  TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_message_type_support_handle = {
  0,
  &TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_Result)() {
  if (!TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_message_type_support_handle.typesupport_identifier) {
    TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifPatternMoveTo_Result__rosidl_typesupport_introspection_c__TifPatternMoveTo_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__TifPatternMoveTo_Feedback__init(message_memory);
}

void TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_fini_function(void * message_memory)
{
  hqv_public_interface__action__TifPatternMoveTo_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_message_member_array[1] = {
  {
    "prog",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_Feedback, prog),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_message_members = {
  "hqv_public_interface__action",  // message namespace
  "TifPatternMoveTo_Feedback",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__action__TifPatternMoveTo_Feedback),
  TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_message_member_array,  // message members
  TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_message_type_support_handle = {
  0,
  &TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_Feedback)() {
  if (!TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_message_type_support_handle.typesupport_identifier) {
    TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifPatternMoveTo_Feedback__rosidl_typesupport_introspection_c__TifPatternMoveTo_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "hqv_public_interface/action/tif_pattern_move_to.h"
// Member `goal`
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__init(message_memory);
}

void TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_fini_function(void * message_memory)
{
  hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_members = {
  "hqv_public_interface__action",  // message namespace
  "TifPatternMoveTo_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request),
  TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_member_array,  // message members
  TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_type_support_handle = {
  0,
  &TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_SendGoal_Request)() {
  TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_Goal)();
  if (!TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifPatternMoveTo_SendGoal_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__init(message_memory);
}

void TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_fini_function(void * message_memory)
{
  hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_members = {
  "hqv_public_interface__action",  // message namespace
  "TifPatternMoveTo_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response),
  TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_member_array,  // message members
  TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_type_support_handle = {
  0,
  &TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_SendGoal_Response)() {
  TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifPatternMoveTo_SendGoal_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_service_members = {
  "hqv_public_interface__action",  // service namespace
  "TifPatternMoveTo_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_service_type_support_handle = {
  0,
  &hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_SendGoal)() {
  if (!hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_SendGoal_Response)()->data;
  }

  return &hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__init(message_memory);
}

void TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_fini_function(void * message_memory)
{
  hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_members = {
  "hqv_public_interface__action",  // message namespace
  "TifPatternMoveTo_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__action__TifPatternMoveTo_GetResult_Request),
  TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_member_array,  // message members
  TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_type_support_handle = {
  0,
  &TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_GetResult_Request)() {
  TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifPatternMoveTo_GetResult_Request__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "hqv_public_interface/action/tif_pattern_move_to.h"
// Member `result`
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__init(message_memory);
}

void TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_fini_function(void * message_memory)
{
  hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_members = {
  "hqv_public_interface__action",  // message namespace
  "TifPatternMoveTo_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response),
  TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_member_array,  // message members
  TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_type_support_handle = {
  0,
  &TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_GetResult_Response)() {
  TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_Result)();
  if (!TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifPatternMoveTo_GetResult_Response__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_service_members = {
  "hqv_public_interface__action",  // service namespace
  "TifPatternMoveTo_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_service_type_support_handle = {
  0,
  &hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_GetResult)() {
  if (!hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_GetResult_Response)()->data;
  }

  return &hqv_public_interface__action__detail__tif_pattern_move_to__rosidl_typesupport_introspection_c__TifPatternMoveTo_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "hqv_public_interface/action/tif_pattern_move_to.h"
// Member `feedback`
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__init(message_memory);
}

void TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_fini_function(void * message_memory)
{
  hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_members = {
  "hqv_public_interface__action",  // message namespace
  "TifPatternMoveTo_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage),
  TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_member_array,  // message members
  TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_type_support_handle = {
  0,
  &TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_FeedbackMessage)() {
  TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, TifPatternMoveTo_Feedback)();
  if (!TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifPatternMoveTo_FeedbackMessage__rosidl_typesupport_introspection_c__TifPatternMoveTo_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
