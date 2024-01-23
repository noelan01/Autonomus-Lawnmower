// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:msg/MowerSlamFeedback.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/msg/detail/mower_slam_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
hqv_public_interface__msg__MowerSlamFeedback__init(hqv_public_interface__msg__MowerSlamFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // feedback
  // feedback_data_1
  // feedback_data_2
  return true;
}

void
hqv_public_interface__msg__MowerSlamFeedback__fini(hqv_public_interface__msg__MowerSlamFeedback * msg)
{
  if (!msg) {
    return;
  }
  // feedback
  // feedback_data_1
  // feedback_data_2
}

hqv_public_interface__msg__MowerSlamFeedback *
hqv_public_interface__msg__MowerSlamFeedback__create()
{
  hqv_public_interface__msg__MowerSlamFeedback * msg = (hqv_public_interface__msg__MowerSlamFeedback *)malloc(sizeof(hqv_public_interface__msg__MowerSlamFeedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__msg__MowerSlamFeedback));
  bool success = hqv_public_interface__msg__MowerSlamFeedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__msg__MowerSlamFeedback__destroy(hqv_public_interface__msg__MowerSlamFeedback * msg)
{
  if (msg) {
    hqv_public_interface__msg__MowerSlamFeedback__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__msg__MowerSlamFeedback__Sequence__init(hqv_public_interface__msg__MowerSlamFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__msg__MowerSlamFeedback * data = NULL;
  if (size) {
    data = (hqv_public_interface__msg__MowerSlamFeedback *)calloc(size, sizeof(hqv_public_interface__msg__MowerSlamFeedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__msg__MowerSlamFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__msg__MowerSlamFeedback__fini(&data[i - 1]);
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
hqv_public_interface__msg__MowerSlamFeedback__Sequence__fini(hqv_public_interface__msg__MowerSlamFeedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__msg__MowerSlamFeedback__fini(&array->data[i]);
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

hqv_public_interface__msg__MowerSlamFeedback__Sequence *
hqv_public_interface__msg__MowerSlamFeedback__Sequence__create(size_t size)
{
  hqv_public_interface__msg__MowerSlamFeedback__Sequence * array = (hqv_public_interface__msg__MowerSlamFeedback__Sequence *)malloc(sizeof(hqv_public_interface__msg__MowerSlamFeedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__msg__MowerSlamFeedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__msg__MowerSlamFeedback__Sequence__destroy(hqv_public_interface__msg__MowerSlamFeedback__Sequence * array)
{
  if (array) {
    hqv_public_interface__msg__MowerSlamFeedback__Sequence__fini(array);
  }
  free(array);
}
