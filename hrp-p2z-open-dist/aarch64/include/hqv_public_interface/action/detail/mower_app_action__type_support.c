// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:action/MowerAppAction.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/action/detail/mower_app_action__functions.h"
#include "hqv_public_interface/action/detail/mower_app_action__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__MowerAppAction_Goal__init(message_memory);
}

void MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_fini_function(void * message_memory)
{
  hqv_public_interface__action__MowerAppAction_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_message_member_array[1] = {
  {
    "action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__MowerAppAction_Goal, action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_message_members = {
  "hqv_public_interface__action",  // message namespace
  "MowerAppAction_Goal",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__action__MowerAppAction_Goal),
  MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_message_member_array,  // message members
  MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_message_type_support_handle = {
  0,
  &MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_Goal)() {
  if (!MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_message_type_support_handle.typesupport_identifier) {
    MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerAppAction_Goal__rosidl_typesupport_introspection_c__MowerAppAction_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__MowerAppAction_Result__init(message_memory);
}

void MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_fini_function(void * message_memory)
{
  hqv_public_interface__action__MowerAppAction_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_message_member_array[1] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__MowerAppAction_Result, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_message_members = {
  "hqv_public_interface__action",  // message namespace
  "MowerAppAction_Result",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__action__MowerAppAction_Result),
  MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_message_member_array,  // message members
  MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_message_type_support_handle = {
  0,
  &MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_Result)() {
  if (!MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_message_type_support_handle.typesupport_identifier) {
    MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerAppAction_Result__rosidl_typesupport_introspection_c__MowerAppAction_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__MowerAppAction_Feedback__init(message_memory);
}

void MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_fini_function(void * message_memory)
{
  hqv_public_interface__action__MowerAppAction_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_message_member_array[1] = {
  {
    "progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__MowerAppAction_Feedback, progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_message_members = {
  "hqv_public_interface__action",  // message namespace
  "MowerAppAction_Feedback",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__action__MowerAppAction_Feedback),
  MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_message_member_array,  // message members
  MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_message_type_support_handle = {
  0,
  &MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_Feedback)() {
  if (!MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_message_type_support_handle.typesupport_identifier) {
    MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerAppAction_Feedback__rosidl_typesupport_introspection_c__MowerAppAction_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "hqv_public_interface/action/mower_app_action.h"
// Member `goal`
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__MowerAppAction_SendGoal_Request__init(message_memory);
}

void MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_fini_function(void * message_memory)
{
  hqv_public_interface__action__MowerAppAction_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__MowerAppAction_SendGoal_Request, goal_id),  // bytes offset in struct
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
    offsetof(hqv_public_interface__action__MowerAppAction_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_members = {
  "hqv_public_interface__action",  // message namespace
  "MowerAppAction_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__action__MowerAppAction_SendGoal_Request),
  MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_member_array,  // message members
  MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_type_support_handle = {
  0,
  &MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_SendGoal_Request)() {
  MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_Goal)();
  if (!MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerAppAction_SendGoal_Request__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__MowerAppAction_SendGoal_Response__init(message_memory);
}

void MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_fini_function(void * message_memory)
{
  hqv_public_interface__action__MowerAppAction_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__MowerAppAction_SendGoal_Response, accepted),  // bytes offset in struct
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
    offsetof(hqv_public_interface__action__MowerAppAction_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_members = {
  "hqv_public_interface__action",  // message namespace
  "MowerAppAction_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__action__MowerAppAction_SendGoal_Response),
  MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_member_array,  // message members
  MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_type_support_handle = {
  0,
  &MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_SendGoal_Response)() {
  MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerAppAction_SendGoal_Response__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_service_members = {
  "hqv_public_interface__action",  // service namespace
  "MowerAppAction_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_service_type_support_handle = {
  0,
  &hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_SendGoal)() {
  if (!hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_SendGoal_Response)()->data;
  }

  return &hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"


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

void MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__MowerAppAction_GetResult_Request__init(message_memory);
}

void MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_fini_function(void * message_memory)
{
  hqv_public_interface__action__MowerAppAction_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__MowerAppAction_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_members = {
  "hqv_public_interface__action",  // message namespace
  "MowerAppAction_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__action__MowerAppAction_GetResult_Request),
  MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_member_array,  // message members
  MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_type_support_handle = {
  0,
  &MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_GetResult_Request)() {
  MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerAppAction_GetResult_Request__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "hqv_public_interface/action/mower_app_action.h"
// Member `result`
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__MowerAppAction_GetResult_Response__init(message_memory);
}

void MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_fini_function(void * message_memory)
{
  hqv_public_interface__action__MowerAppAction_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__MowerAppAction_GetResult_Response, status),  // bytes offset in struct
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
    offsetof(hqv_public_interface__action__MowerAppAction_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_members = {
  "hqv_public_interface__action",  // message namespace
  "MowerAppAction_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__action__MowerAppAction_GetResult_Response),
  MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_member_array,  // message members
  MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_type_support_handle = {
  0,
  &MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_GetResult_Response)() {
  MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_Result)();
  if (!MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerAppAction_GetResult_Response__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_service_members = {
  "hqv_public_interface__action",  // service namespace
  "MowerAppAction_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_service_type_support_handle = {
  0,
  &hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_GetResult)() {
  if (!hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_GetResult_Response)()->data;
  }

  return &hqv_public_interface__action__detail__mower_app_action__rosidl_typesupport_introspection_c__MowerAppAction_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__functions.h"
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "hqv_public_interface/action/mower_app_action.h"
// Member `feedback`
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__action__MowerAppAction_FeedbackMessage__init(message_memory);
}

void MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_fini_function(void * message_memory)
{
  hqv_public_interface__action__MowerAppAction_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__action__MowerAppAction_FeedbackMessage, goal_id),  // bytes offset in struct
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
    offsetof(hqv_public_interface__action__MowerAppAction_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_members = {
  "hqv_public_interface__action",  // message namespace
  "MowerAppAction_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__action__MowerAppAction_FeedbackMessage),
  MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_member_array,  // message members
  MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_type_support_handle = {
  0,
  &MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_FeedbackMessage)() {
  MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, action, MowerAppAction_Feedback)();
  if (!MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerAppAction_FeedbackMessage__rosidl_typesupport_introspection_c__MowerAppAction_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
