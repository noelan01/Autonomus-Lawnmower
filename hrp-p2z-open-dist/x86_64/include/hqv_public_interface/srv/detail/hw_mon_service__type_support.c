// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:srv/HwMonService.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/srv/detail/hw_mon_service__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/srv/detail/hw_mon_service__functions.h"
#include "hqv_public_interface/srv/detail/hw_mon_service__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__HwMonService_Request__init(message_memory);
}

void HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_fini_function(void * message_memory)
{
  hqv_public_interface__srv__HwMonService_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__HwMonService_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "HwMonService_Request",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__srv__HwMonService_Request),
  HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_message_member_array,  // message members
  HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_message_type_support_handle = {
  0,
  &HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, HwMonService_Request)() {
  if (!HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_message_type_support_handle.typesupport_identifier) {
    HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &HwMonService_Request__rosidl_typesupport_introspection_c__HwMonService_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hqv_public_interface/srv/detail/hw_mon_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hqv_public_interface/srv/detail/hw_mon_service__functions.h"
// already included above
// #include "hqv_public_interface/srv/detail/hw_mon_service__struct.h"


// Include directives for member types
// Member `hw_monitor`
#include "hqv_public_interface/msg/hw_mon.h"
// Member `hw_monitor`
#include "hqv_public_interface/msg/detail/hw_mon__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__srv__HwMonService_Response__init(message_memory);
}

void HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_fini_function(void * message_memory)
{
  hqv_public_interface__srv__HwMonService_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_member_array[1] = {
  {
    "hw_monitor",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__srv__HwMonService_Response, hw_monitor),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_members = {
  "hqv_public_interface__srv",  // message namespace
  "HwMonService_Response",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface__srv__HwMonService_Response),
  HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_member_array,  // message members
  HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_type_support_handle = {
  0,
  &HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, HwMonService_Response)() {
  HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, HwMon)();
  if (!HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_type_support_handle.typesupport_identifier) {
    HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &HwMonService_Response__rosidl_typesupport_introspection_c__HwMonService_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hqv_public_interface/srv/detail/hw_mon_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_service_members = {
  "hqv_public_interface__srv",  // service namespace
  "HwMonService",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_Request_message_type_support_handle,
  NULL  // response message
  // hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_Response_message_type_support_handle
};

static rosidl_service_type_support_t hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_service_type_support_handle = {
  0,
  &hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, HwMonService_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, HwMonService_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, HwMonService)() {
  if (!hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_service_type_support_handle.typesupport_identifier) {
    hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, HwMonService_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, srv, HwMonService_Response)()->data;
  }

  return &hqv_public_interface__srv__detail__hw_mon_service__rosidl_typesupport_introspection_c__HwMonService_service_type_support_handle;
}
