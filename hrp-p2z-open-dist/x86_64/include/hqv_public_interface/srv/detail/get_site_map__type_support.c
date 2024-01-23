// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:srv/GetSiteMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/srv/detail/get_site_map__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/srv/detail/get_site_map__functions.h"
#include "hqv_public_interface/srv/detail/get_site_map__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__GetSiteMap_Request__init(message_memory);
}

void GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_fini_function(void * message_memory)
{
  hqv_public_interface__srv__GetSiteMap_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_member_array[1] = {
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__GetSiteMap_Request, request),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "GetSiteMap_Request",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__srv__GetSiteMap_Request),
  GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_member_array,  // message members
  GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_type_support_handle = {
  0,
  &GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, GetSiteMap_Request)() {
  if (!GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_type_support_handle.typesupport_identifier) {
    GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetSiteMap_Request__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/srv/detail/get_site_map__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/srv/detail/get_site_map__functions.h"
// already included above
// #include "hqv_public_interface/srv/detail/get_site_map__struct.h"


// Include directives for member types
// Member `sitename`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__GetSiteMap_Response__init(message_memory);
}

void GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_fini_function(void * message_memory)
{
  hqv_public_interface__srv__GetSiteMap_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_member_array[4] = {
  {
    "loaded",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__GetSiteMap_Response, loaded),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__GetSiteMap_Response, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sitename",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__GetSiteMap_Response, sitename),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__GetSiteMap_Response, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "GetSiteMap_Response",  // message name
  4,  // number of fields
  sizeof(hqv_public_interface__srv__GetSiteMap_Response),
  GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_member_array,  // message members
  GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_type_support_handle = {
  0,
  &GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, GetSiteMap_Response)() {
  if (!GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_type_support_handle.typesupport_identifier) {
    GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetSiteMap_Response__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/srv/detail/get_site_map__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_service_members = {
  "hqv_public_interface__srv",  // service namespace
  "GetSiteMap",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_service_type_support_handle = {
  0,
  &hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, GetSiteMap_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, GetSiteMap_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, GetSiteMap)() {
  if (!hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, GetSiteMap_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, GetSiteMap_Response)()->data;
  }

  return &hqv_public_interface__srv__detail__get_site_map__rosidl_typesupport_introspection_c__GetSiteMap_service_type_support_handle;
}
