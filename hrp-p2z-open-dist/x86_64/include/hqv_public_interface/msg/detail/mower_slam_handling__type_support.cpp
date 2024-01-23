// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hqv_public_interface:msg/MowerSlamHandling.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hqv_public_interface/msg/detail/mower_slam_handling__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hqv_public_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MowerSlamHandling_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hqv_public_interface::msg::MowerSlamHandling(_init);
}

void MowerSlamHandling_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hqv_public_interface::msg::MowerSlamHandling *>(message_memory);
  typed_message->~MowerSlamHandling();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MowerSlamHandling_message_member_array[1] = {
  {
    "slam_handle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MowerSlamHandling, slam_handle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MowerSlamHandling_message_members = {
  "hqv_public_interface::msg",  // message namespace
  "MowerSlamHandling",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface::msg::MowerSlamHandling),
  MowerSlamHandling_message_member_array,  // message members
  MowerSlamHandling_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerSlamHandling_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MowerSlamHandling_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MowerSlamHandling_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace hqv_public_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hqv_public_interface::msg::MowerSlamHandling>()
{
  return &::hqv_public_interface::msg::rosidl_typesupport_introspection_cpp::MowerSlamHandling_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hqv_public_interface, msg, MowerSlamHandling)() {
  return &::hqv_public_interface::msg::rosidl_typesupport_introspection_cpp::MowerSlamHandling_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
