// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:srv/MowerSlamMapHandling.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/srv/detail/mower_slam_map_handling__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/srv/detail/mower_slam_map_handling__functions.h"
#include "hqv_public_interface/srv/detail/mower_slam_map_handling__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__MowerSlamMapHandling_Request__init(message_memory);
}

void MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_fini_function(void * message_memory)
{
  hqv_public_interface__srv__MowerSlamMapHandling_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_member_array[2] = {
  {
    "message_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__MowerSlamMapHandling_Request, message_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "map_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__MowerSlamMapHandling_Request, map_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "MowerSlamMapHandling_Request",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Request),
  MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_member_array,  // message members
  MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_type_support_handle = {
  0,
  &MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, MowerSlamMapHandling_Request)() {
  if (!MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_type_support_handle.typesupport_identifier) {
    MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerSlamMapHandling_Request__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/srv/detail/mower_slam_map_handling__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/srv/detail/mower_slam_map_handling__functions.h"
// already included above
// #include "hqv_public_interface/srv/detail/mower_slam_map_handling__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__MowerSlamMapHandling_Response__init(message_memory);
}

void MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_fini_function(void * message_memory)
{
  hqv_public_interface__srv__MowerSlamMapHandling_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_member_array[2] = {
  {
    "map_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__MowerSlamMapHandling_Response, map_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "map_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    10,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__MowerSlamMapHandling_Response, map_ids),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "MowerSlamMapHandling_Response",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__srv__MowerSlamMapHandling_Response),
  MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_member_array,  // message members
  MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_type_support_handle = {
  0,
  &MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, MowerSlamMapHandling_Response)() {
  if (!MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_type_support_handle.typesupport_identifier) {
    MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MowerSlamMapHandling_Response__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/srv/detail/mower_slam_map_handling__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_service_members = {
  "hqv_public_interface__srv",  // service namespace
  "MowerSlamMapHandling",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_service_type_support_handle = {
  0,
  &hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, MowerSlamMapHandling_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, MowerSlamMapHandling_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, MowerSlamMapHandling)() {
  if (!hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, MowerSlamMapHandling_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, MowerSlamMapHandling_Response)()->data;
  }

  return &hqv_public_interface__srv__detail__mower_slam_map_handling__rosidl_typesupport_introspection_c__MowerSlamMapHandling_service_type_support_handle;
}
