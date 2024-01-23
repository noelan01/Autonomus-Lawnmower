// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hqv_public_interface:action/TifPatternMoveTo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__FUNCTIONS_H_
#define HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hqv_public_interface/msg/rosidl_generator_c__visibility_control.h"

#include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"

/// Initialize action/TifPatternMoveTo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__action__TifPatternMoveTo_Goal
 * )) before or use
 * hqv_public_interface__action__TifPatternMoveTo_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_Goal__init(hqv_public_interface__action__TifPatternMoveTo_Goal * msg);

/// Finalize action/TifPatternMoveTo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Goal__fini(hqv_public_interface__action__TifPatternMoveTo_Goal * msg);

/// Create action/TifPatternMoveTo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__action__TifPatternMoveTo_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_Goal *
hqv_public_interface__action__TifPatternMoveTo_Goal__create();

/// Destroy action/TifPatternMoveTo message.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Goal__destroy(hqv_public_interface__action__TifPatternMoveTo_Goal * msg);


/// Initialize array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__action__TifPatternMoveTo_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence__init(hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence * array, size_t size);

/// Finalize array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence__fini(hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence * array);

/// Create array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence *
hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence__create(size_t size);

/// Destroy array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence__destroy(hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence * array);

/// Initialize action/TifPatternMoveTo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__action__TifPatternMoveTo_Result
 * )) before or use
 * hqv_public_interface__action__TifPatternMoveTo_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_Result__init(hqv_public_interface__action__TifPatternMoveTo_Result * msg);

/// Finalize action/TifPatternMoveTo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Result__fini(hqv_public_interface__action__TifPatternMoveTo_Result * msg);

/// Create action/TifPatternMoveTo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__action__TifPatternMoveTo_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_Result *
hqv_public_interface__action__TifPatternMoveTo_Result__create();

/// Destroy action/TifPatternMoveTo message.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Result__destroy(hqv_public_interface__action__TifPatternMoveTo_Result * msg);


/// Initialize array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__action__TifPatternMoveTo_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_Result__Sequence__init(hqv_public_interface__action__TifPatternMoveTo_Result__Sequence * array, size_t size);

/// Finalize array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Result__Sequence__fini(hqv_public_interface__action__TifPatternMoveTo_Result__Sequence * array);

/// Create array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__action__TifPatternMoveTo_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_Result__Sequence *
hqv_public_interface__action__TifPatternMoveTo_Result__Sequence__create(size_t size);

/// Destroy array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Result__Sequence__destroy(hqv_public_interface__action__TifPatternMoveTo_Result__Sequence * array);

/// Initialize action/TifPatternMoveTo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__action__TifPatternMoveTo_Feedback
 * )) before or use
 * hqv_public_interface__action__TifPatternMoveTo_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_Feedback__init(hqv_public_interface__action__TifPatternMoveTo_Feedback * msg);

/// Finalize action/TifPatternMoveTo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Feedback__fini(hqv_public_interface__action__TifPatternMoveTo_Feedback * msg);

/// Create action/TifPatternMoveTo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__action__TifPatternMoveTo_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_Feedback *
hqv_public_interface__action__TifPatternMoveTo_Feedback__create();

/// Destroy action/TifPatternMoveTo message.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Feedback__destroy(hqv_public_interface__action__TifPatternMoveTo_Feedback * msg);


/// Initialize array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__action__TifPatternMoveTo_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence__init(hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence * array, size_t size);

/// Finalize array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence__fini(hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence * array);

/// Create array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence *
hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence__create(size_t size);

/// Destroy array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence__destroy(hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence * array);

/// Initialize action/TifPatternMoveTo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request
 * )) before or use
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__init(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request * msg);

/// Finalize action/TifPatternMoveTo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__fini(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request * msg);

/// Create action/TifPatternMoveTo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request *
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__create();

/// Destroy action/TifPatternMoveTo message.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__destroy(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request * msg);


/// Initialize array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence__init(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence__fini(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence * array);

/// Create array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence *
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence__destroy(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence * array);

/// Initialize action/TifPatternMoveTo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response
 * )) before or use
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__init(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response * msg);

/// Finalize action/TifPatternMoveTo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__fini(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response * msg);

/// Create action/TifPatternMoveTo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response *
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__create();

/// Destroy action/TifPatternMoveTo message.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__destroy(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response * msg);


/// Initialize array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence__init(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence__fini(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence * array);

/// Create array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence *
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence__destroy(hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence * array);

/// Initialize action/TifPatternMoveTo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Request
 * )) before or use
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__init(hqv_public_interface__action__TifPatternMoveTo_GetResult_Request * msg);

/// Finalize action/TifPatternMoveTo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__fini(hqv_public_interface__action__TifPatternMoveTo_GetResult_Request * msg);

/// Create action/TifPatternMoveTo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request *
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__create();

/// Destroy action/TifPatternMoveTo message.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__destroy(hqv_public_interface__action__TifPatternMoveTo_GetResult_Request * msg);


/// Initialize array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence__init(hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence__fini(hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence * array);

/// Create array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence *
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence__destroy(hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence * array);

/// Initialize action/TifPatternMoveTo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Response
 * )) before or use
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__init(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response * msg);

/// Finalize action/TifPatternMoveTo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__fini(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response * msg);

/// Create action/TifPatternMoveTo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response *
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__create();

/// Destroy action/TifPatternMoveTo message.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__destroy(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response * msg);


/// Initialize array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence__init(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence__fini(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence * array);

/// Create array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence *
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence__destroy(hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence * array);

/// Initialize action/TifPatternMoveTo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage
 * )) before or use
 * hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__init(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage * msg);

/// Finalize action/TifPatternMoveTo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__fini(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage * msg);

/// Create action/TifPatternMoveTo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage *
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__create();

/// Destroy action/TifPatternMoveTo message.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__destroy(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage * msg);


/// Initialize array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the number of elements and calls
 * hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
bool
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence__init(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence__fini(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence * array);

/// Create array of action/TifPatternMoveTo messages.
/**
 * It allocates the memory for the array and calls
 * hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence *
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/TifPatternMoveTo messages.
/**
 * It calls
 * hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hqv_public_interface
void
hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence__destroy(hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__FUNCTIONS_H_
