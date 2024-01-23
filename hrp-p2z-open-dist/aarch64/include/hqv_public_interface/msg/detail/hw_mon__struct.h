// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:msg/HwMon.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/HwMon in the package hqv_public_interface.
typedef struct hqv_public_interface__msg__HwMon
{
  int32_t temperature;
  int32_t cpu_freq;
  int32_t used_memory;
} hqv_public_interface__msg__HwMon;

// Struct for a sequence of hqv_public_interface__msg__HwMon.
typedef struct hqv_public_interface__msg__HwMon__Sequence
{
  hqv_public_interface__msg__HwMon * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__msg__HwMon__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__HW_MON__STRUCT_H_
