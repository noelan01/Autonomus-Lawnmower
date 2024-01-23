// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:msg/MowerCoDriverMoveTo.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/msg/detail/mower_co_driver_move_to__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
hqv_public_interface__msg__MowerCoDriverMoveTo__init(hqv_public_interface__msg__MowerCoDriverMoveTo * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
hqv_public_interface__msg__MowerCoDriverMoveTo__fini(hqv_public_interface__msg__MowerCoDriverMoveTo * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

hqv_public_interface__msg__MowerCoDriverMoveTo *
hqv_public_interface__msg__MowerCoDriverMoveTo__create()
{
  hqv_public_interface__msg__MowerCoDriverMoveTo * msg = (hqv_public_interface__msg__MowerCoDriverMoveTo *)malloc(sizeof(hqv_public_interface__msg__MowerCoDriverMoveTo));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__msg__MowerCoDriverMoveTo));
  bool success = hqv_public_interface__msg__MowerCoDriverMoveTo__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__msg__MowerCoDriverMoveTo__destroy(hqv_public_interface__msg__MowerCoDriverMoveTo * msg)
{
  if (msg) {
    hqv_public_interface__msg__MowerCoDriverMoveTo__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence__init(hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__msg__MowerCoDriverMoveTo * data = NULL;
  if (size) {
    data = (hqv_public_interface__msg__MowerCoDriverMoveTo *)calloc(size, sizeof(hqv_public_interface__msg__MowerCoDriverMoveTo));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__msg__MowerCoDriverMoveTo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__msg__MowerCoDriverMoveTo__fini(&data[i - 1]);
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
hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence__fini(hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__msg__MowerCoDriverMoveTo__fini(&array->data[i]);
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

hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence *
hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence__create(size_t size)
{
  hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence * array = (hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence *)malloc(sizeof(hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence__destroy(hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence * array)
{
  if (array) {
    hqv_public_interface__msg__MowerCoDriverMoveTo__Sequence__fini(array);
  }
  free(array);
}
