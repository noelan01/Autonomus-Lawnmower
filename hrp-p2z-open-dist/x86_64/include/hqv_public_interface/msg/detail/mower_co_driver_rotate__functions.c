// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:msg/MowerCoDriverRotate.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/msg/detail/mower_co_driver_rotate__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
hqv_public_interface__msg__MowerCoDriverRotate__init(hqv_public_interface__msg__MowerCoDriverRotate * msg)
{
  if (!msg) {
    return false;
  }
  // rotateangle
  return true;
}

void
hqv_public_interface__msg__MowerCoDriverRotate__fini(hqv_public_interface__msg__MowerCoDriverRotate * msg)
{
  if (!msg) {
    return;
  }
  // rotateangle
}

hqv_public_interface__msg__MowerCoDriverRotate *
hqv_public_interface__msg__MowerCoDriverRotate__create()
{
  hqv_public_interface__msg__MowerCoDriverRotate * msg = (hqv_public_interface__msg__MowerCoDriverRotate *)malloc(sizeof(hqv_public_interface__msg__MowerCoDriverRotate));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__msg__MowerCoDriverRotate));
  bool success = hqv_public_interface__msg__MowerCoDriverRotate__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__msg__MowerCoDriverRotate__destroy(hqv_public_interface__msg__MowerCoDriverRotate * msg)
{
  if (msg) {
    hqv_public_interface__msg__MowerCoDriverRotate__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__msg__MowerCoDriverRotate__Sequence__init(hqv_public_interface__msg__MowerCoDriverRotate__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__msg__MowerCoDriverRotate * data = NULL;
  if (size) {
    data = (hqv_public_interface__msg__MowerCoDriverRotate *)calloc(size, sizeof(hqv_public_interface__msg__MowerCoDriverRotate));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__msg__MowerCoDriverRotate__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__msg__MowerCoDriverRotate__fini(&data[i - 1]);
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
hqv_public_interface__msg__MowerCoDriverRotate__Sequence__fini(hqv_public_interface__msg__MowerCoDriverRotate__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__msg__MowerCoDriverRotate__fini(&array->data[i]);
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

hqv_public_interface__msg__MowerCoDriverRotate__Sequence *
hqv_public_interface__msg__MowerCoDriverRotate__Sequence__create(size_t size)
{
  hqv_public_interface__msg__MowerCoDriverRotate__Sequence * array = (hqv_public_interface__msg__MowerCoDriverRotate__Sequence *)malloc(sizeof(hqv_public_interface__msg__MowerCoDriverRotate__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__msg__MowerCoDriverRotate__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__msg__MowerCoDriverRotate__Sequence__destroy(hqv_public_interface__msg__MowerCoDriverRotate__Sequence * array)
{
  if (array) {
    hqv_public_interface__msg__MowerCoDriverRotate__Sequence__fini(array);
  }
  free(array);
}
