// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:srv/TifGetFile.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/srv/detail/tif_get_file__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/srv/detail/tif_get_file__functions.h"
#include "hqv_public_interface/srv/detail/tif_get_file__struct.h"


// Include directives for member types
// Member `filename`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__TifGetFile_Request__init(message_memory);
}

void TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_fini_function(void * message_memory)
{
  hqv_public_interface__srv__TifGetFile_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_message_member_array[1] = {
  {
    "filename",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__TifGetFile_Request, filename),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "TifGetFile_Request",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__srv__TifGetFile_Request),
  TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_message_member_array,  // message members
  TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_message_type_support_handle = {
  0,
  &TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifGetFile_Request)() {
  if (!TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_message_type_support_handle.typesupport_identifier) {
    TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifGetFile_Request__rosidl_typesupport_introspection_c__TifGetFile_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/srv/detail/tif_get_file__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/srv/detail/tif_get_file__functions.h"
// already included above
// #include "hqv_public_interface/srv/detail/tif_get_file__struct.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__TifGetFile_Response__init(message_memory);
}

void TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_fini_function(void * message_memory)
{
  hqv_public_interface__srv__TifGetFile_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__TifGetFile_Response, status),  // bytes offset in struct
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
    offsetof(hqv_public_interface__srv__TifGetFile_Response, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "TifGetFile_Response",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface__srv__TifGetFile_Response),
  TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_message_member_array,  // message members
  TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_message_type_support_handle = {
  0,
  &TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifGetFile_Response)() {
  if (!TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_message_type_support_handle.typesupport_identifier) {
    TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifGetFile_Response__rosidl_typesupport_introspection_c__TifGetFile_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/srv/detail/tif_get_file__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_service_members = {
  "hqv_public_interface__srv",  // service namespace
  "TifGetFile",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_service_type_support_handle = {
  0,
  &hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifGetFile_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifGetFile_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifGetFile)() {
  if (!hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifGetFile_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifGetFile_Response)()->data;
  }

  return &hqv_public_interface__srv__detail__tif_get_file__rosidl_typesupport_introspection_c__TifGetFile_service_type_support_handle;
}
