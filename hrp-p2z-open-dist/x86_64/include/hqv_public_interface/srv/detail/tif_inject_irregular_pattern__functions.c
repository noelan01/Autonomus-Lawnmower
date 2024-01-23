// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:srv/TifInjectIrregularPattern.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/srv/detail/tif_inject_irregular_pattern__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
hqv_public_interface__srv__TifInjectIrregularPattern_Request__init(hqv_public_interface__srv__TifInjectIrregularPattern_Request * msg)
{
  if (!msg) {
    return false;
  }
  // set_state
  return true;
}

void
hqv_public_interface__srv__TifInjectIrregularPattern_Request__fini(hqv_public_interface__srv__TifInjectIrregularPattern_Request * msg)
{
  if (!msg) {
    return;
  }
  // set_state
}

hqv_public_interface__srv__TifInjectIrregularPattern_Request *
hqv_public_interface__srv__TifInjectIrregularPattern_Request__create()
{
  hqv_public_interface__srv__TifInjectIrregularPattern_Request * msg = (hqv_public_interface__srv__TifInjectIrregularPattern_Request *)malloc(sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Request));
  bool success = hqv_public_interface__srv__TifInjectIrregularPattern_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__srv__TifInjectIrregularPattern_Request__destroy(hqv_public_interface__srv__TifInjectIrregularPattern_Request * msg)
{
  if (msg) {
    hqv_public_interface__srv__TifInjectIrregularPattern_Request__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence__init(hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__srv__TifInjectIrregularPattern_Request * data = NULL;
  if (size) {
    data = (hqv_public_interface__srv__TifInjectIrregularPattern_Request *)calloc(size, sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__srv__TifInjectIrregularPattern_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__srv__TifInjectIrregularPattern_Request__fini(&data[i - 1]);
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
hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence__fini(hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__srv__TifInjectIrregularPattern_Request__fini(&array->data[i]);
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

hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence *
hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence__create(size_t size)
{
  hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence * array = (hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence *)malloc(sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence__destroy(hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence * array)
{
  if (array) {
    hqv_public_interface__srv__TifInjectIrregularPattern_Request__Sequence__fini(array);
  }
  free(array);
}


bool
hqv_public_interface__srv__TifInjectIrregularPattern_Response__init(hqv_public_interface__srv__TifInjectIrregularPattern_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  return true;
}

void
hqv_public_interface__srv__TifInjectIrregularPattern_Response__fini(hqv_public_interface__srv__TifInjectIrregularPattern_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
}

hqv_public_interface__srv__TifInjectIrregularPattern_Response *
hqv_public_interface__srv__TifInjectIrregularPattern_Response__create()
{
  hqv_public_interface__srv__TifInjectIrregularPattern_Response * msg = (hqv_public_interface__srv__TifInjectIrregularPattern_Response *)malloc(sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Response));
  bool success = hqv_public_interface__srv__TifInjectIrregularPattern_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__srv__TifInjectIrregularPattern_Response__destroy(hqv_public_interface__srv__TifInjectIrregularPattern_Response * msg)
{
  if (msg) {
    hqv_public_interface__srv__TifInjectIrregularPattern_Response__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence__init(hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__srv__TifInjectIrregularPattern_Response * data = NULL;
  if (size) {
    data = (hqv_public_interface__srv__TifInjectIrregularPattern_Response *)calloc(size, sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__srv__TifInjectIrregularPattern_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__srv__TifInjectIrregularPattern_Response__fini(&data[i - 1]);
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
hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence__fini(hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__srv__TifInjectIrregularPattern_Response__fini(&array->data[i]);
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

hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence *
hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence__create(size_t size)
{
  hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence * array = (hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence *)malloc(sizeof(hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence__destroy(hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence * array)
{
  if (array) {
    hqv_public_interface__srv__TifInjectIrregularPattern_Response__Sequence__fini(array);
  }
  free(array);
}
