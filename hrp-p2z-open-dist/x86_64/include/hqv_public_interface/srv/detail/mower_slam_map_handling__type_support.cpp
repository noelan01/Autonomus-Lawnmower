// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hqv_public_interface:srv/MowerSlamMapHandling.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hqv_public_interface/srv/detail/mower_slam_map_handling__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hqv_public_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void MowerSlamMapHandling_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hqv_public_interface::srv::MowerSlamMapHandling_Request(_init);
}

void MowerSlamMapHandling_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hqv_public_interface::srv::MowerSlamMapHandling_Request *>(message_memory);
  typed_message->~MowerSlamMapHandling_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MowerSlamMapHandling_Request_message_member_array[2] = {
  {
    "message_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::srv::MowerSlamMapHandling_Request, message_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "map_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::srv::MowerSlamMapHandling_Request, map_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MowerSlamMapHandling_Request_message_members = {
  "hqv_public_interface::srv",  // message namespace
  "MowerSlamMapHandling_Request",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface::srv::MowerSlamMapHandling_Request),
  MowerSlamMapHandling_Request_message_member_array,  // message members
  MowerSlamMapHandling_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerSlamMapHandling_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MowerSlamMapHandling_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MowerSlamMapHandling_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace hqv_public_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hqv_public_interface::srv::MowerSlamMapHandling_Request>()
{
  return &::hqv_public_interface::srv::rosidl_typesupport_introspection_cpp::MowerSlamMapHandling_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hqv_public_interface, srv, MowerSlamMapHandling_Request)() {
  return &::hqv_public_interface::srv::rosidl_typesupport_introspection_cpp::MowerSlamMapHandling_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hqv_public_interface/srv/detail/mower_slam_map_handling__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hqv_public_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void MowerSlamMapHandling_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hqv_public_interface::srv::MowerSlamMapHandling_Response(_init);
}

void MowerSlamMapHandling_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hqv_public_interface::srv::MowerSlamMapHandling_Response *>(message_memory);
  typed_message->~MowerSlamMapHandling_Response();
}

size_t size_function__MowerSlamMapHandling_Response__map_ids(const void * untyped_member)
{
  (void)untyped_member;
  return 10;
}

const void * get_const_function__MowerSlamMapHandling_Response__map_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint32_t, 10> *>(untyped_member);
  return &member[index];
}

void * get_function__MowerSlamMapHandling_Response__map_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint32_t, 10> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MowerSlamMapHandling_Response_message_member_array[2] = {
  {
    "map_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::srv::MowerSlamMapHandling_Response, map_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "map_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    10,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::srv::MowerSlamMapHandling_Response, map_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__MowerSlamMapHandling_Response__map_ids,  // size() function pointer
    get_const_function__MowerSlamMapHandling_Response__map_ids,  // get_const(index) function pointer
    get_function__MowerSlamMapHandling_Response__map_ids,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MowerSlamMapHandling_Response_message_members = {
  "hqv_public_interface::srv",  // message namespace
  "MowerSlamMapHandling_Response",  // message name
  2,  // number of fields
  sizeof(hqv_public_interface::srv::MowerSlamMapHandling_Response),
  MowerSlamMapHandling_Response_message_member_array,  // message members
  MowerSlamMapHandling_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerSlamMapHandling_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MowerSlamMapHandling_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MowerSlamMapHandling_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace hqv_public_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hqv_public_interface::srv::MowerSlamMapHandling_Response>()
{
  return &::hqv_public_interface::srv::rosidl_typesupport_introspection_cpp::MowerSlamMapHandling_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hqv_public_interface, srv, MowerSlamMapHandling_Response)() {
  return &::hqv_public_interface::srv::rosidl_typesupport_introspection_cpp::MowerSlamMapHandling_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "hqv_public_interface/srv/detail/mower_slam_map_handling__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace hqv_public_interface
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers MowerSlamMapHandling_service_members = {
  "hqv_public_interface::srv",  // service namespace
  "MowerSlamMapHandling",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<hqv_public_interface::srv::MowerSlamMapHandling>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t MowerSlamMapHandling_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MowerSlamMapHandling_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace hqv_public_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<hqv_public_interface::srv::MowerSlamMapHandling>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::hqv_public_interface::srv::rosidl_typesupport_introspection_cpp::MowerSlamMapHandling_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::hqv_public_interface::srv::MowerSlamMapHandling_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::hqv_public_interface::srv::MowerSlamMapHandling_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hqv_public_interface, srv, MowerSlamMapHandling)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<hqv_public_interface::srv::MowerSlamMapHandling>();
}

#ifdef __cplusplus
}
#endif
