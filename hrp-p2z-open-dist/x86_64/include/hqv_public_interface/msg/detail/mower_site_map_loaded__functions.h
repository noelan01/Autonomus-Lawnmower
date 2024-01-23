// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hqv_public_interface:msg/MowerSiteMapLoaded.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__FUNCTIONS_H_
#define HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hqv_public_interface/msg/rosidl_generator_c__visibility_control.h"

#include "hqv_public_interface/msg/detail/mower_site_map_loaded__struct.h"

/// Initialize msg/MowerSiteMapLoaded message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__msg__MowerSiteMapLoaded
 * )) before or use
 * hqv_public_interface__msg__MowerSiteMapLoaded__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__msg__MowerSiteMapLoaded__init(hqv_public_interface__msg__MowerSiteMapLoaded * msg);

/// Finalize msg/MowerSiteMapLoaded message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__msg__MowerSiteMapLoaded__fini(hqv_public_interface__msg__MowerSiteMapLoaded * msg);

/// Create msg/MowerSiteMapLoaded message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__msg__MowerSiteMapLoaded__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__msg__MowerSiteMapLoaded *
hqv_public_interface__msg__MowerSiteMapLoaded__create();

/// Destroy msg/MowerSiteMapLoaded message.
/**
 * It calls
 * hqv_public_interface__msg__MowerSiteMapLoaded__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__msg__MowerSiteMapLoaded__destroy(hqv_public_interface__msg__MowerSiteMapLoaded * msg);


/// Initialize array of msg/MowerSiteMapLoaded messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__msg__MowerSiteMapLoaded__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__msg__MowerSiteMapLoaded__Sequence__init(hqv_public_interface__msg__MowerSiteMapLoaded__Sequence * array, size_t size);

/// Finalize array of msg/MowerSiteMapLoaded messages.
/**
 * It calls
 * hqv_public_interface__msg__MowerSiteMapLoaded__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__msg__MowerSiteMapLoaded__Sequence__fini(hqv_public_interface__msg__MowerSiteMapLoaded__Sequence * array);

/// Create array of msg/MowerSiteMapLoaded messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__msg__MowerSiteMapLoaded__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__msg__MowerSiteMapLoaded__Sequence *
hqv_public_interface__msg__MowerSiteMapLoaded__Sequence__create(size_t size);

/// Destroy array of msg/MowerSiteMapLoaded messages.
/**
 * It calls
 * hqv_public_interface__msg__MowerSiteMapLoaded__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__msg__MowerSiteMapLoaded__Sequence__destroy(hqv_public_interface__msg__MowerSiteMapLoaded__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__MSG__DETAIL__MOWER_SITE_MAP_LOADED__FUNCTIONS_H_
