// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hqv_public_interface:msg/MowerGnssRtkRelPosNEDRaw.idl
// generated code does not contain a copyright notice
#include "hqv_public_interface/msg/detail/mower_gnss_rtk_rel_pos_ned_raw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__init(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__fini(msg);
    return false;
  }
  // time_ms
  // version
  // reserved0
  // ref_station_id
  // i_tow
  // rel_pos_n
  // rel_pos_e
  // rel_pos_d
  // rel_pos_length
  // rel_pos_heading
  // reserved1_0
  // reserved1_1
  // reserved1_2
  // reserved1_3
  // rel_pos_hpn
  // rel_pos_hpe
  // rel_pos_hpd
  // rel_pos_hp_length
  // acc_n
  // acc_e
  // acc_d
  // acc_length
  // acc_heading
  // reserved2_0
  // reserved2_1
  // reserved2_2
  // reserved2_3
  // flags
  return true;
}

void
hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__fini(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // time_ms
  // version
  // reserved0
  // ref_station_id
  // i_tow
  // rel_pos_n
  // rel_pos_e
  // rel_pos_d
  // rel_pos_length
  // rel_pos_heading
  // reserved1_0
  // reserved1_1
  // reserved1_2
  // reserved1_3
  // rel_pos_hpn
  // rel_pos_hpe
  // rel_pos_hpd
  // rel_pos_hp_length
  // acc_n
  // acc_e
  // acc_d
  // acc_length
  // acc_heading
  // reserved2_0
  // reserved2_1
  // reserved2_2
  // reserved2_3
  // flags
}

hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw *
hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__create()
{
  hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw * msg = (hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw *)malloc(sizeof(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw));
  bool success = hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__destroy(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw * msg)
{
  if (msg) {
    hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__fini(msg);
  }
  free(msg);
}


bool
hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence__init(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw * data = NULL;
  if (size) {
    data = (hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw *)calloc(size, sizeof(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__fini(&data[i - 1]);
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
hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence__fini(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__fini(&array->data[i]);
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

hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence *
hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence__create(size_t size)
{
  hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence * array = (hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence *)malloc(sizeof(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence__destroy(hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence * array)
{
  if (array) {
    hqv_public_interface__msg__MowerGnssRtkRelPosNEDRaw__Sequence__fini(array);
  }
  free(array);
}
