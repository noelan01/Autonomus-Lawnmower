// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:srv/GetSiteMap.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/srv/detail/get_site_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
hqv_public_interface__srv__GetSiteMap_Request__init(hqv_public_interface__srv__GetSiteMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // request
  return true;
}

void
hqv_public_interface__srv__GetSiteMap_Request__fini(hqv_public_interface__srv__GetSiteMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // request
}

hqv_public_interface__srv__GetSiteMap_Request *
hqv_public_interface__srv__GetSiteMap_Request__create()
{
  hqv_public_interface__srv__GetSiteMap_Request * msg = (hqv_public_interface__srv__GetSiteMap_Request *)malloc(sizeof(hqv_public_interface__srv__GetSiteMap_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__srv__GetSiteMap_Request));
  bool success = hqv_public_interface__srv__GetSiteMap_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__srv__GetSiteMap_Request__destroy(hqv_public_interface__srv__GetSiteMap_Request * msg)
{
  if (msg) {
    hqv_public_interface__srv__GetSiteMap_Request__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__srv__GetSiteMap_Request__Sequence__init(hqv_public_interface__srv__GetSiteMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__srv__GetSiteMap_Request * data = NULL;
  if (size) {
    data = (hqv_public_interface__srv__GetSiteMap_Request *)calloc(size, sizeof(hqv_public_interface__srv__GetSiteMap_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__srv__GetSiteMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__srv__GetSiteMap_Request__fini(&data[i - 1]);
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
hqv_public_interface__srv__GetSiteMap_Request__Sequence__fini(hqv_public_interface__srv__GetSiteMap_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__srv__GetSiteMap_Request__fini(&array->data[i]);
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

hqv_public_interface__srv__GetSiteMap_Request__Sequence *
hqv_public_interface__srv__GetSiteMap_Request__Sequence__create(size_t size)
{
  hqv_public_interface__srv__GetSiteMap_Request__Sequence * array = (hqv_public_interface__srv__GetSiteMap_Request__Sequence *)malloc(sizeof(hqv_public_interface__srv__GetSiteMap_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__srv__GetSiteMap_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__srv__GetSiteMap_Request__Sequence__destroy(hqv_public_interface__srv__GetSiteMap_Request__Sequence * array)
{
  if (array) {
    hqv_public_interface__srv__GetSiteMap_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `sitename`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
hqv_public_interface__srv__GetSiteMap_Response__init(hqv_public_interface__srv__GetSiteMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // loaded
  // valid
  // sitename
  if (!rosidl_runtime_c__String__init(&msg->sitename)) {
    hqv_public_interface__srv__GetSiteMap_Response__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__octet__Sequence__init(&msg->data, 0)) {
    hqv_public_interface__srv__GetSiteMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
hqv_public_interface__srv__GetSiteMap_Response__fini(hqv_public_interface__srv__GetSiteMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // loaded
  // valid
  // sitename
  rosidl_runtime_c__String__fini(&msg->sitename);
  // data
  rosidl_runtime_c__octet__Sequence__fini(&msg->data);
}

hqv_public_interface__srv__GetSiteMap_Response *
hqv_public_interface__srv__GetSiteMap_Response__create()
{
  hqv_public_interface__srv__GetSiteMap_Response * msg = (hqv_public_interface__srv__GetSiteMap_Response *)malloc(sizeof(hqv_public_interface__srv__GetSiteMap_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__srv__GetSiteMap_Response));
  bool success = hqv_public_interface__srv__GetSiteMap_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__srv__GetSiteMap_Response__destroy(hqv_public_interface__srv__GetSiteMap_Response * msg)
{
  if (msg) {
    hqv_public_interface__srv__GetSiteMap_Response__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__srv__GetSiteMap_Response__Sequence__init(hqv_public_interface__srv__GetSiteMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__srv__GetSiteMap_Response * data = NULL;
  if (size) {
    data = (hqv_public_interface__srv__GetSiteMap_Response *)calloc(size, sizeof(hqv_public_interface__srv__GetSiteMap_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__srv__GetSiteMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__srv__GetSiteMap_Response__fini(&data[i - 1]);
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
hqv_public_interface__srv__GetSiteMap_Response__Sequence__fini(hqv_public_interface__srv__GetSiteMap_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__srv__GetSiteMap_Response__fini(&array->data[i]);
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

hqv_public_interface__srv__GetSiteMap_Response__Sequence *
hqv_public_interface__srv__GetSiteMap_Response__Sequence__create(size_t size)
{
  hqv_public_interface__srv__GetSiteMap_Response__Sequence * array = (hqv_public_interface__srv__GetSiteMap_Response__Sequence *)malloc(sizeof(hqv_public_interface__srv__GetSiteMap_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__srv__GetSiteMap_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__srv__GetSiteMap_Response__Sequence__destroy(hqv_public_interface__srv__GetSiteMap_Response__Sequence * array)
{
  if (array) {
    hqv_public_interface__srv__GetSiteMap_Response__Sequence__fini(array);
  }
  free(array);
}
