// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hqv_public_interface:msg/MapInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hqv_public_interface/msg/detail/map_info__rosidl_typesupport_introspection_c.h"
#include "hqv_public_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hqv_public_interface/msg/detail/map_info__functions.h"
#include "hqv_public_interface/msg/detail/map_info__struct.h"


// Include directives for member types
// Member `map_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `cs_pos`
// Member `pc_pos`
#include "geometry_msgs/msg/point32.h"
// Member `cs_pos`
// Member `pc_pos`
#include "geometry_msgs/msg/detail/point32__rosidl_typesupport_introspection_c.h"
// Member `charging_station`
// Member `working_areas`
// Member `transport_areas`
// Member `forbidden_areas`
// Member `hidden_areas`
// Member `transport_paths`
// Member `poi_paths`
#include "hqv_public_interface/msg/map_shape.h"
// Member `charging_station`
// Member `working_areas`
// Member `transport_areas`
// Member `forbidden_areas`
// Member `hidden_areas`
// Member `transport_paths`
// Member `poi_paths`
#include "hqv_public_interface/msg/detail/map_shape__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MapInfo__rosidl_typesupport_introspection_c__MapInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hqv_public_interface__msg__MapInfo__init(message_memory);
}

void MapInfo__rosidl_typesupport_introspection_c__MapInfo_fini_function(void * message_memory)
{
  hqv_public_interface__msg__MapInfo__fini(message_memory);
}

size_t MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__working_areas(
  const void * untyped_member)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return member->size;
}

const void * MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__working_areas(
  const void * untyped_member, size_t index)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__working_areas(
  void * untyped_member, size_t index)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__working_areas(
  void * untyped_member, size_t size)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  hqv_public_interface__msg__MapShape__Sequence__fini(member);
  return hqv_public_interface__msg__MapShape__Sequence__init(member, size);
}

size_t MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__transport_areas(
  const void * untyped_member)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return member->size;
}

const void * MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__transport_areas(
  const void * untyped_member, size_t index)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__transport_areas(
  void * untyped_member, size_t index)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__transport_areas(
  void * untyped_member, size_t size)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  hqv_public_interface__msg__MapShape__Sequence__fini(member);
  return hqv_public_interface__msg__MapShape__Sequence__init(member, size);
}

size_t MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__forbidden_areas(
  const void * untyped_member)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return member->size;
}

const void * MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__forbidden_areas(
  const void * untyped_member, size_t index)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__forbidden_areas(
  void * untyped_member, size_t index)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__forbidden_areas(
  void * untyped_member, size_t size)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  hqv_public_interface__msg__MapShape__Sequence__fini(member);
  return hqv_public_interface__msg__MapShape__Sequence__init(member, size);
}

size_t MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__hidden_areas(
  const void * untyped_member)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return member->size;
}

const void * MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__hidden_areas(
  const void * untyped_member, size_t index)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__hidden_areas(
  void * untyped_member, size_t index)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__hidden_areas(
  void * untyped_member, size_t size)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  hqv_public_interface__msg__MapShape__Sequence__fini(member);
  return hqv_public_interface__msg__MapShape__Sequence__init(member, size);
}

size_t MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__transport_paths(
  const void * untyped_member)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return member->size;
}

const void * MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__transport_paths(
  const void * untyped_member, size_t index)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__transport_paths(
  void * untyped_member, size_t index)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__transport_paths(
  void * untyped_member, size_t size)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  hqv_public_interface__msg__MapShape__Sequence__fini(member);
  return hqv_public_interface__msg__MapShape__Sequence__init(member, size);
}

size_t MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__poi_paths(
  const void * untyped_member)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return member->size;
}

const void * MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__poi_paths(
  const void * untyped_member, size_t index)
{
  const hqv_public_interface__msg__MapShape__Sequence * member =
    (const hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__poi_paths(
  void * untyped_member, size_t index)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__poi_paths(
  void * untyped_member, size_t size)
{
  hqv_public_interface__msg__MapShape__Sequence * member =
    (hqv_public_interface__msg__MapShape__Sequence *)(untyped_member);
  hqv_public_interface__msg__MapShape__Sequence__fini(member);
  return hqv_public_interface__msg__MapShape__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[10] = {
  {
    "map_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, map_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cs_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, cs_pos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pc_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, pc_pos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "charging_station",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, charging_station),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "working_areas",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, working_areas),  // bytes offset in struct
    NULL,  // default value
    MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__working_areas,  // size() function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__working_areas,  // get_const(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__working_areas,  // get(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__working_areas  // resize(index) function pointer
  },
  {
    "transport_areas",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, transport_areas),  // bytes offset in struct
    NULL,  // default value
    MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__transport_areas,  // size() function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__transport_areas,  // get_const(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__transport_areas,  // get(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__transport_areas  // resize(index) function pointer
  },
  {
    "forbidden_areas",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, forbidden_areas),  // bytes offset in struct
    NULL,  // default value
    MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__forbidden_areas,  // size() function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__forbidden_areas,  // get_const(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__forbidden_areas,  // get(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__forbidden_areas  // resize(index) function pointer
  },
  {
    "hidden_areas",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, hidden_areas),  // bytes offset in struct
    NULL,  // default value
    MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__hidden_areas,  // size() function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__hidden_areas,  // get_const(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__hidden_areas,  // get(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__hidden_areas  // resize(index) function pointer
  },
  {
    "transport_paths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, transport_paths),  // bytes offset in struct
    NULL,  // default value
    MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__transport_paths,  // size() function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__transport_paths,  // get_const(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__transport_paths,  // get(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__transport_paths  // resize(index) function pointer
  },
  {
    "poi_paths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hqv_public_interface__msg__MapInfo, poi_paths),  // bytes offset in struct
    NULL,  // default value
    MapInfo__rosidl_typesupport_introspection_c__size_function__MapShape__poi_paths,  // size() function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_const_function__MapShape__poi_paths,  // get_const(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__get_function__MapShape__poi_paths,  // get(index) function pointer
    MapInfo__rosidl_typesupport_introspection_c__resize_function__MapShape__poi_paths  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_members = {
  "hqv_public_interface__msg",  // message namespace
  "MapInfo",  // message name
  10,  // number of fields
  sizeof(hqv_public_interface__msg__MapInfo),
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array,  // message members
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_type_support_handle = {
  0,
  &MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hqv_public_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MapInfo)() {
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point32)();
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MapShape)();
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MapShape)();
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MapShape)();
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MapShape)();
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MapShape)();
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MapShape)();
  MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hqv_public_interface, msg, MapShape)();
  if (!MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_type_support_handle.typesupport_identifier) {
    MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MapInfo__rosidl_typesupport_introspection_c__MapInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
