// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:msg/GnssRtkCollection.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/msg/detail/gnss_rtk_collection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
hqv_public_interface__msg__GnssRtkCollection__init(hqv_public_interface__msg__GnssRtkCollection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hqv_public_interface__msg__GnssRtkCollection__fini(msg);
    return false;
  }
  // time_ms
  // ref_station_id
  // i_tow
  // rel_pos_n
  // rel_pos_e
  // rel_pos_d
  // rel_pos_hpn
  // rel_pos_hpe
  // rel_pos_hpd
  // acc_n
  // acc_e
  // acc_d
  // flags
  // year
  // month
  // day
  // hour
  // min
  // sec
  // validity_flags
  // t_acc
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
  return true;
}

void
hqv_public_interface__msg__GnssRtkCollection__fini(hqv_public_interface__msg__GnssRtkCollection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // time_ms
  // ref_station_id
  // i_tow
  // rel_pos_n
  // rel_pos_e
  // rel_pos_d
  // rel_pos_hpn
  // rel_pos_hpe
  // rel_pos_hpd
  // acc_n
  // acc_e
  // acc_d
  // flags
  // year
  // month
  // day
  // hour
  // min
  // sec
  // validity_flags
  // t_acc
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
}

hqv_public_interface__msg__GnssRtkCollection *
hqv_public_interface__msg__GnssRtkCollection__create()
{
  hqv_public_interface__msg__GnssRtkCollection * msg = (hqv_public_interface__msg__GnssRtkCollection *)malloc(sizeof(hqv_public_interface__msg__GnssRtkCollection));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__msg__GnssRtkCollection));
  bool success = hqv_public_interface__msg__GnssRtkCollection__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__msg__GnssRtkCollection__destroy(hqv_public_interface__msg__GnssRtkCollection * msg)
{
  if (msg) {
    hqv_public_interface__msg__GnssRtkCollection__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__msg__GnssRtkCollection__Sequence__init(hqv_public_interface__msg__GnssRtkCollection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__msg__GnssRtkCollection * data = NULL;
  if (size) {
    data = (hqv_public_interface__msg__GnssRtkCollection *)calloc(size, sizeof(hqv_public_interface__msg__GnssRtkCollection));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__msg__GnssRtkCollection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__msg__GnssRtkCollection__fini(&data[i - 1]);
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
hqv_public_interface__msg__GnssRtkCollection__Sequence__fini(hqv_public_interface__msg__GnssRtkCollection__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__msg__GnssRtkCollection__fini(&array->data[i]);
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

hqv_public_interface__msg__GnssRtkCollection__Sequence *
hqv_public_interface__msg__GnssRtkCollection__Sequence__create(size_t size)
{
  hqv_public_interface__msg__GnssRtkCollection__Sequence * array = (hqv_public_interface__msg__GnssRtkCollection__Sequence *)malloc(sizeof(hqv_public_interface__msg__GnssRtkCollection__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__msg__GnssRtkCollection__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__msg__GnssRtkCollection__Sequence__destroy(hqv_public_interface__msg__GnssRtkCollection__Sequence * array)
{
  if (array) {
    hqv_public_interface__msg__GnssRtkCollection__Sequence__fini(array);
  }
  free(array);
}
