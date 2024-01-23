// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:msg/MowerSiteMapChecksumUpdate.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/msg/detail/mower_site_map_checksum_update__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `description`
#include "rosidl_runtime_c/string_functions.h"

bool
hqv_public_interface__msg__MowerSiteMapChecksumUpdate__init(hqv_public_interface__msg__MowerSiteMapChecksumUpdate * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hqv_public_interface__msg__MowerSiteMapChecksumUpdate__fini(msg);
    return false;
  }
  // checksum
  // description
  if (!rosidl_runtime_c__String__init(&msg->description)) {
    hqv_public_interface__msg__MowerSiteMapChecksumUpdate__fini(msg);
    return false;
  }
  return true;
}

void
hqv_public_interface__msg__MowerSiteMapChecksumUpdate__fini(hqv_public_interface__msg__MowerSiteMapChecksumUpdate * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // checksum
  // description
  rosidl_runtime_c__String__fini(&msg->description);
}

hqv_public_interface__msg__MowerSiteMapChecksumUpdate *
hqv_public_interface__msg__MowerSiteMapChecksumUpdate__create()
{
  hqv_public_interface__msg__MowerSiteMapChecksumUpdate * msg = (hqv_public_interface__msg__MowerSiteMapChecksumUpdate *)malloc(sizeof(hqv_public_interface__msg__MowerSiteMapChecksumUpdate));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__msg__MowerSiteMapChecksumUpdate));
  bool success = hqv_public_interface__msg__MowerSiteMapChecksumUpdate__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__msg__MowerSiteMapChecksumUpdate__destroy(hqv_public_interface__msg__MowerSiteMapChecksumUpdate * msg)
{
  if (msg) {
    hqv_public_interface__msg__MowerSiteMapChecksumUpdate__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence__init(hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__msg__MowerSiteMapChecksumUpdate * data = NULL;
  if (size) {
    data = (hqv_public_interface__msg__MowerSiteMapChecksumUpdate *)calloc(size, sizeof(hqv_public_interface__msg__MowerSiteMapChecksumUpdate));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__msg__MowerSiteMapChecksumUpdate__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__msg__MowerSiteMapChecksumUpdate__fini(&data[i - 1]);
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
hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence__fini(hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__msg__MowerSiteMapChecksumUpdate__fini(&array->data[i]);
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

hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence *
hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence__create(size_t size)
{
  hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence * array = (hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence *)malloc(sizeof(hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence__destroy(hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence * array)
{
  if (array) {
    hqv_public_interface__msg__MowerSiteMapChecksumUpdate__Sequence__fini(array);
  }
  free(array);
}
