// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hqv_public_interface:msg/MowerLedStatus.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__FUNCTIONS_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hqv_public_interface/msg/rosidl_generator_c__visibility_control.h"

#include "hqv_public_interface/msg/detail/mower_led_status__struct.h"

/// Initialize msg/MowerLedStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__msg__MowerLedStatus
 * )) before or use
 * hqv_public_interface__msg__MowerLedStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__msg__MowerLedStatus__init(hqv_public_interface__msg__MowerLedStatus * msg);

/// Finalize msg/MowerLedStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__msg__MowerLedStatus__fini(hqv_public_interface__msg__MowerLedStatus * msg);

/// Create msg/MowerLedStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__msg__MowerLedStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__msg__MowerLedStatus *
hqv_public_interface__msg__MowerLedStatus__create();

/// Destroy msg/MowerLedStatus message.
/**
 * It calls
 * hqv_public_interface__msg__MowerLedStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__msg__MowerLedStatus__destroy(hqv_public_interface__msg__MowerLedStatus * msg);


/// Initialize array of msg/MowerLedStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__msg__MowerLedStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__msg__MowerLedStatus__Sequence__init(hqv_public_interface__msg__MowerLedStatus__Sequence * array, size_t size);

/// Finalize array of msg/MowerLedStatus messages.
/**
 * It calls
 * hqv_public_interface__msg__MowerLedStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__msg__MowerLedStatus__Sequence__fini(hqv_public_interface__msg__MowerLedStatus__Sequence * array);

/// Create array of msg/MowerLedStatus messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__msg__MowerLedStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__msg__MowerLedStatus__Sequence *
hqv_public_interface__msg__MowerLedStatus__Sequence__create(size_t size);

/// Destroy array of msg/MowerLedStatus messages.
/**
 * It calls
 * hqv_public_interface__msg__MowerLedStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__msg__MowerLedStatus__Sequence__destroy(hqv_public_interface__msg__MowerLedStatus__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_LED_STATUS__FUNCTIONS_H_
