// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:msg/MapInfo.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/msg/detail/map_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `map_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `cs_pos`
// Member `pc_pos`
#include "geometry_msgs/msg/detail/point32__functions.h"
// Member `charging_station`
// Member `working_areas`
// Member `transport_areas`
// Member `forbidden_areas`
// Member `hidden_areas`
// Member `transport_paths`
// Member `poi_paths`
#include "hqv_public_interface/msg/detail/map_shape__functions.h"

bool
hqv_public_interface__msg__MapInfo__init(hqv_public_interface__msg__MapInfo * msg)
{
  if (!msg) {
    return false;
  }
  // map_name
  if (!rosidl_runtime_c__String__init(&msg->map_name)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // cs_pos
  if (!geometry_msgs__msg__Point32__init(&msg->cs_pos)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // pc_pos
  if (!geometry_msgs__msg__Point32__init(&msg->pc_pos)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // charging_station
  if (!hqv_public_interface__msg__MapShape__init(&msg->charging_station)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // working_areas
  if (!hqv_public_interface__msg__MapShape__Sequence__init(&msg->working_areas, 0)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // transport_areas
  if (!hqv_public_interface__msg__MapShape__Sequence__init(&msg->transport_areas, 0)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // forbidden_areas
  if (!hqv_public_interface__msg__MapShape__Sequence__init(&msg->forbidden_areas, 0)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // hidden_areas
  if (!hqv_public_interface__msg__MapShape__Sequence__init(&msg->hidden_areas, 0)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // transport_paths
  if (!hqv_public_interface__msg__MapShape__Sequence__init(&msg->transport_paths, 0)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  // poi_paths
  if (!hqv_public_interface__msg__MapShape__Sequence__init(&msg->poi_paths, 0)) {
    hqv_public_interface__msg__MapInfo__fini(msg);
    return false;
  }
  return true;
}

void
hqv_public_interface__msg__MapInfo__fini(hqv_public_interface__msg__MapInfo * msg)
{
  if (!msg) {
    return;
  }
  // map_name
  rosidl_runtime_c__String__fini(&msg->map_name);
  // cs_pos
  geometry_msgs__msg__Point32__fini(&msg->cs_pos);
  // pc_pos
  geometry_msgs__msg__Point32__fini(&msg->pc_pos);
  // charging_station
  hqv_public_interface__msg__MapShape__fini(&msg->charging_station);
  // working_areas
  hqv_public_interface__msg__MapShape__Sequence__fini(&msg->working_areas);
  // transport_areas
  hqv_public_interface__msg__MapShape__Sequence__fini(&msg->transport_areas);
  // forbidden_areas
  hqv_public_interface__msg__MapShape__Sequence__fini(&msg->forbidden_areas);
  // hidden_areas
  hqv_public_interface__msg__MapShape__Sequence__fini(&msg->hidden_areas);
  // transport_paths
  hqv_public_interface__msg__MapShape__Sequence__fini(&msg->transport_paths);
  // poi_paths
  hqv_public_interface__msg__MapShape__Sequence__fini(&msg->poi_paths);
}

hqv_public_interface__msg__MapInfo *
hqv_public_interface__msg__MapInfo__create()
{
  hqv_public_interface__msg__MapInfo * msg = (hqv_public_interface__msg__MapInfo *)malloc(sizeof(hqv_public_interface__msg__MapInfo));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__msg__MapInfo));
  bool success = hqv_public_interface__msg__MapInfo__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__msg__MapInfo__destroy(hqv_public_interface__msg__MapInfo * msg)
{
  if (msg) {
    hqv_public_interface__msg__MapInfo__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__msg__MapInfo__Sequence__init(hqv_public_interface__msg__MapInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__msg__MapInfo * data = NULL;
  if (size) {
    data = (hqv_public_interface__msg__MapInfo *)calloc(size, sizeof(hqv_public_interface__msg__MapInfo));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__msg__MapInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__msg__MapInfo__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hqv_public_interface__msg__MapInfo__Sequence__fini(hqv_public_interface__msg__MapInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__msg__MapInfo__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hqv_public_interface__msg__MapInfo__Sequence *
hqv_public_interface__msg__MapInfo__Sequence__create(size_t size)
{
  hqv_public_interface__msg__MapInfo__Sequence * array = (hqv_public_interface__msg__MapInfo__Sequence *)malloc(sizeof(hqv_public_interface__msg__MapInfo__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__msg__MapInfo__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__msg__MapInfo__Sequence__destroy(hqv_public_interface__msg__MapInfo__Sequence * array)
{
  if (array) {
    hqv_public_interface__msg__MapInfo__Sequence__fini(array);
  }
  free(array);
}
