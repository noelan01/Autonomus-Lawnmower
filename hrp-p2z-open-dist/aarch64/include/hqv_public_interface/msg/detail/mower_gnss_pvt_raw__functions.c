// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:msg/MowerGnssPvtRaw.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/msg/detail/mower_gnss_pvt_raw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
hqv_public_interface__msg__MowerGnssPvtRaw__init(hqv_public_interface__msg__MowerGnssPvtRaw * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hqv_public_interface__msg__MowerGnssPvtRaw__fini(msg);
    return false;
  }
  // time_ms
  // i_tow
  // year
  // month
  // day
  // hour
  // min
  // sec
  // validity_flags
  // t_acc
  // nano
  // fix_type
  // fix_status_flags
  // flags2
  // num_sv
  // lon
  // lat
  // height
  // h_msl
  // h_acc
  // v_acc
  // vel_n
  // vel_e
  // vel_d
  // g_speed
  // head_mot
  // s_acc
  // head_acc
  // p_dop
  // flags3
  // reserved0_0
  // reserved0_1
  // reserved0_2
  // reserved0_3
  // reserved0_4
  // head_veh
  // mag_dec
  // mag_acc
  return true;
}

void
hqv_public_interface__msg__MowerGnssPvtRaw__fini(hqv_public_interface__msg__MowerGnssPvtRaw * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // time_ms
  // i_tow
  // year
  // month
  // day
  // hour
  // min
  // sec
  // validity_flags
  // t_acc
  // nano
  // fix_type
  // fix_status_flags
  // flags2
  // num_sv
  // lon
  // lat
  // height
  // h_msl
  // h_acc
  // v_acc
  // vel_n
  // vel_e
  // vel_d
  // g_speed
  // head_mot
  // s_acc
  // head_acc
  // p_dop
  // flags3
  // reserved0_0
  // reserved0_1
  // reserved0_2
  // reserved0_3
  // reserved0_4
  // head_veh
  // mag_dec
  // mag_acc
}

hqv_public_interface__msg__MowerGnssPvtRaw *
hqv_public_interface__msg__MowerGnssPvtRaw__create()
{
  hqv_public_interface__msg__MowerGnssPvtRaw * msg = (hqv_public_interface__msg__MowerGnssPvtRaw *)malloc(sizeof(hqv_public_interface__msg__MowerGnssPvtRaw));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__msg__MowerGnssPvtRaw));
  bool success = hqv_public_interface__msg__MowerGnssPvtRaw__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__msg__MowerGnssPvtRaw__destroy(hqv_public_interface__msg__MowerGnssPvtRaw * msg)
{
  if (msg) {
    hqv_public_interface__msg__MowerGnssPvtRaw__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__msg__MowerGnssPvtRaw__Sequence__init(hqv_public_interface__msg__MowerGnssPvtRaw__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__msg__MowerGnssPvtRaw * data = NULL;
  if (size) {
    data = (hqv_public_interface__msg__MowerGnssPvtRaw *)calloc(size, sizeof(hqv_public_interface__msg__MowerGnssPvtRaw));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__msg__MowerGnssPvtRaw__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__msg__MowerGnssPvtRaw__fini(&data[i - 1]);
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
hqv_public_interface__msg__MowerGnssPvtRaw__Sequence__fini(hqv_public_interface__msg__MowerGnssPvtRaw__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__msg__MowerGnssPvtRaw__fini(&array->data[i]);
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

hqv_public_interface__msg__MowerGnssPvtRaw__Sequence *
hqv_public_interface__msg__MowerGnssPvtRaw__Sequence__create(size_t size)
{
  hqv_public_interface__msg__MowerGnssPvtRaw__Sequence * array = (hqv_public_interface__msg__MowerGnssPvtRaw__Sequence *)malloc(sizeof(hqv_public_interface__msg__MowerGnssPvtRaw__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__msg__MowerGnssPvtRaw__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__msg__MowerGnssPvtRaw__Sequence__destroy(hqv_public_interface__msg__MowerGnssPvtRaw__Sequence * array)
{
  if (array) {
    hqv_public_interface__msg__MowerGnssPvtRaw__Sequence__fini(array);
  }
  free(array);
}
