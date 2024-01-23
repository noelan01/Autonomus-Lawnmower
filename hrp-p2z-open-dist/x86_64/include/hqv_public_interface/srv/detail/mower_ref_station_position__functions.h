// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hqv_public_interface:srv/MowerRefStationPosition.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__FUNCTIONS_H_
#define HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hqv_public_interface/msg/rosidl_generator_c__visibility_control.h"

#include "hqv_public_interface/srv/detail/mower_ref_station_position__struct.h"

/// Initialize srv/MowerRefStationPosition message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__srv__MowerRefStationPosition_Request
 * )) before or use
 * hqv_public_interface__srv__MowerRefStationPosition_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__srv__MowerRefStationPosition_Request__init(hqv_public_interface__srv__MowerRefStationPosition_Request * msg);

/// Finalize srv/MowerRefStationPosition message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__srv__MowerRefStationPosition_Request__fini(hqv_public_interface__srv__MowerRefStationPosition_Request * msg);

/// Create srv/MowerRefStationPosition message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__srv__MowerRefStationPosition_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__srv__MowerRefStationPosition_Request *
hqv_public_interface__srv__MowerRefStationPosition_Request__create();

/// Destroy srv/MowerRefStationPosition message.
/**
 * It calls
 * hqv_public_interface__srv__MowerRefStationPosition_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__srv__MowerRefStationPosition_Request__destroy(hqv_public_interface__srv__MowerRefStationPosition_Request * msg);


/// Initialize array of srv/MowerRefStationPosition messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__srv__MowerRefStationPosition_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence__init(hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence * array, size_t size);

/// Finalize array of srv/MowerRefStationPosition messages.
/**
 * It calls
 * hqv_public_interface__srv__MowerRefStationPosition_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence__fini(hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence * array);

/// Create array of srv/MowerRefStationPosition messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence *
hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence__create(size_t size);

/// Destroy array of srv/MowerRefStationPosition messages.
/**
 * It calls
 * hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence__destroy(hqv_public_interface__srv__MowerRefStationPosition_Request__Sequence * array);

/// Initialize srv/MowerRefStationPosition message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__srv__MowerRefStationPosition_Response
 * )) before or use
 * hqv_public_interface__srv__MowerRefStationPosition_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__srv__MowerRefStationPosition_Response__init(hqv_public_interface__srv__MowerRefStationPosition_Response * msg);

/// Finalize srv/MowerRefStationPosition message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__srv__MowerRefStationPosition_Response__fini(hqv_public_interface__srv__MowerRefStationPosition_Response * msg);

/// Create srv/MowerRefStationPosition message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__srv__MowerRefStationPosition_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__srv__MowerRefStationPosition_Response *
hqv_public_interface__srv__MowerRefStationPosition_Response__create();

/// Destroy srv/MowerRefStationPosition message.
/**
 * It calls
 * hqv_public_interface__srv__MowerRefStationPosition_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__srv__MowerRefStationPosition_Response__destroy(hqv_public_interface__srv__MowerRefStationPosition_Response * msg);


/// Initialize array of srv/MowerRefStationPosition messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__srv__MowerRefStationPosition_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence__init(hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence * array, size_t size);

/// Finalize array of srv/MowerRefStationPosition messages.
/**
 * It calls
 * hqv_public_interface__srv__MowerRefStationPosition_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence__fini(hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence * array);

/// Create array of srv/MowerRefStationPosition messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence *
hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence__create(size_t size);

/// Destroy array of srv/MowerRefStationPosition messages.
/**
 * It calls
 * hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence__destroy(hqv_public_interface__srv__MowerRefStationPosition_Response__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__SRV__DETAIL__MOWER_REF_STATION_POSITION__FUNCTIONS_H_
