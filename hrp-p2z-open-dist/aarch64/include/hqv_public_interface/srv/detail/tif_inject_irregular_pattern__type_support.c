// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:srv/TifInjectIrregularPattern.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__functions.h"
#include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__TifInjectIrregularPattern_Request__init(message_memory);
}

void TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_fini_function(void * message_memory)
{
  hqv_public_interface__srv__TifInjectIrregularPattern_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_member_array[1] = {
  {
    "set_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__TifInjectIrregularPattern_Request, set_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "TifInjectIrregularPattern_Request",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Request),
  TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_member_array,  // message members
  TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_type_support_handle = {
  0,
  &TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifInjectIrregularPattern_Request)() {
  if (!TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_type_support_handle.typesupport_identifier) {
    TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifInjectIrregularPattern_Request__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__functions.h"
// already included above
// #include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__TifInjectIrregularPattern_Response__init(message_memory);
}

void TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_fini_function(void * message_memory)
{
  hqv_public_interface__srv__TifInjectIrregularPattern_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_member_array[1] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__TifInjectIrregularPattern_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "TifInjectIrregularPattern_Response",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Response),
  TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_member_array,  // message members
  TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_type_support_handle = {
  0,
  &TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifInjectIrregularPattern_Response)() {
  if (!TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_type_support_handle.typesupport_identifier) {
    TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TifInjectIrregularPattern_Response__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_service_members = {
  "hqv_public_interface__srv",  // service namespace
  "TifInjectIrregularPattern",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_service_type_support_handle = {
  0,
  &hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifInjectIrregularPattern_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifInjectIrregularPattern_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifInjectIrregularPattern)() {
  if (!hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifInjectIrregularPattern_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, TifInjectIrregularPattern_Response)()->data;
  }

  return &hqv_public_interface__srv__detail__tif_inject_irregular_pattern__rosidl_typesupport_introspection_c__TifInjectIrregularPattern_service_type_support_handle;
}
