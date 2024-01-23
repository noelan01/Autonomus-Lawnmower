// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hqv_public_interface:msg/MapInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hqv_public_interface/msg/detail/map_info__struct.hpp"
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

void MapInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hqv_public_interface::msg::MapInfo(_init);
}

void MapInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hqv_public_interface::msg::MapInfo *>(message_memory);
  typed_message->~MapInfo();
}

size_t size_function__MapInfo__working_areas(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MapInfo__working_areas(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void * get_function__MapInfo__working_areas(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void resize_function__MapInfo__working_areas(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MapInfo__transport_areas(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MapInfo__transport_areas(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void * get_function__MapInfo__transport_areas(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void resize_function__MapInfo__transport_areas(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MapInfo__forbidden_areas(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MapInfo__forbidden_areas(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void * get_function__MapInfo__forbidden_areas(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void resize_function__MapInfo__forbidden_areas(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MapInfo__hidden_areas(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MapInfo__hidden_areas(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void * get_function__MapInfo__hidden_areas(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void resize_function__MapInfo__hidden_areas(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MapInfo__transport_paths(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MapInfo__transport_paths(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void * get_function__MapInfo__transport_paths(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void resize_function__MapInfo__transport_paths(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MapInfo__poi_paths(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MapInfo__poi_paths(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void * get_function__MapInfo__poi_paths(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  return &member[index];
}

void resize_function__MapInfo__poi_paths(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<hqv_public_interface::msg::MapShape> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MapInfo_message_member_array[10] = {
  {
    "map_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, map_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cs_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, cs_pos),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pc_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point32>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, pc_pos),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "charging_station",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hqv_public_interface::msg::MapShape>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, charging_station),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "working_areas",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hqv_public_interface::msg::MapShape>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, working_areas),  // bytes offset in struct
    nullptr,  // default value
    size_function__MapInfo__working_areas,  // size() function pointer
    get_const_function__MapInfo__working_areas,  // get_const(index) function pointer
    get_function__MapInfo__working_areas,  // get(index) function pointer
    resize_function__MapInfo__working_areas  // resize(index) function pointer
  },
  {
    "transport_areas",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hqv_public_interface::msg::MapShape>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, transport_areas),  // bytes offset in struct
    nullptr,  // default value
    size_function__MapInfo__transport_areas,  // size() function pointer
    get_const_function__MapInfo__transport_areas,  // get_const(index) function pointer
    get_function__MapInfo__transport_areas,  // get(index) function pointer
    resize_function__MapInfo__transport_areas  // resize(index) function pointer
  },
  {
    "forbidden_areas",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hqv_public_interface::msg::MapShape>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, forbidden_areas),  // bytes offset in struct
    nullptr,  // default value
    size_function__MapInfo__forbidden_areas,  // size() function pointer
    get_const_function__MapInfo__forbidden_areas,  // get_const(index) function pointer
    get_function__MapInfo__forbidden_areas,  // get(index) function pointer
    resize_function__MapInfo__forbidden_areas  // resize(index) function pointer
  },
  {
    "hidden_areas",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hqv_public_interface::msg::MapShape>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, hidden_areas),  // bytes offset in struct
    nullptr,  // default value
    size_function__MapInfo__hidden_areas,  // size() function pointer
    get_const_function__MapInfo__hidden_areas,  // get_const(index) function pointer
    get_function__MapInfo__hidden_areas,  // get(index) function pointer
    resize_function__MapInfo__hidden_areas  // resize(index) function pointer
  },
  {
    "transport_paths",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hqv_public_interface::msg::MapShape>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, transport_paths),  // bytes offset in struct
    nullptr,  // default value
    size_function__MapInfo__transport_paths,  // size() function pointer
    get_const_function__MapInfo__transport_paths,  // get_const(index) function pointer
    get_function__MapInfo__transport_paths,  // get(index) function pointer
    resize_function__MapInfo__transport_paths  // resize(index) function pointer
  },
  {
    "poi_paths",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hqv_public_interface::msg::MapShape>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface::msg::MapInfo, poi_paths),  // bytes offset in struct
    nullptr,  // default value
    size_function__MapInfo__poi_paths,  // size() function pointer
    get_const_function__MapInfo__poi_paths,  // get_const(index) function pointer
    get_function__MapInfo__poi_paths,  // get(index) function pointer
    resize_function__MapInfo__poi_paths  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MapInfo_message_members = {
  "hqv_public_interface::msg",  // message namespace
  "MapInfo",  // message name
  10,  // number of fields
  sizeof(hqv_public_interface::msg::MapInfo),
  MapInfo_message_member_array,  // message members
  MapInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  MapInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MapInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MapInfo_message_members,
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
get_message_type_support_handle<hqv_public_interface::msg::MapInfo>()
{
  return &::hqv_public_interface::msg::rosidl_typesupport_introspection_cpp::MapInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hqv_public_interface, msg, MapInfo)() {
  return &::hqv_public_interface::msg::rosidl_typesupport_introspection_cpp::MapInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
