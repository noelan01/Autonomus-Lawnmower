// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hqv_public_interface:msg/MowerCoDriverTurn.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hqv_public_interface/msg/detail/mower_co_driver_turn__struct.hpp"
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

void MowerCoDriverTurn_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hqv_public_interface::msg::MowerCoDriverTurn(_init);
}

void MowerCoDriverTurn_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hqv_public_interface::msg::MowerCoDriverTurn *>(message_memory);
  typed_message->~MowerCoDriverTurn();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MowerCoDriverTurn_message_member_array[1] = {
  {
    "turnangle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MowerCoDriverTurn, turnangle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MowerCoDriverTurn_message_members = {
  "hqv_public_interface::msg",  // message namespace
  "MowerCoDriverTurn",  // message name
  1,  // number of fields
  sizeof(hqv_public_interface::msg::MowerCoDriverTurn),
  MowerCoDriverTurn_message_member_array,  // message members
  MowerCoDriverTurn_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerCoDriverTurn_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MowerCoDriverTurn_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MowerCoDriverTurn_message_members,
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
get_message_type_support_handle<hqv_public_interface::msg::MowerCoDriverTurn>()
{
  return &::hqv_public_interface::msg::rosidl_typesupport_introspection_cpp::MowerCoDriverTurn_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hqv_public_interface, msg, MowerCoDriverTurn)() {
  return &::hqv_public_interface::msg::rosidl_typesupport_introspection_cpp::MowerCoDriverTurn_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
