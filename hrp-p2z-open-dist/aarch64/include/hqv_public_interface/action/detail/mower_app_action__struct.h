// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:action/MowerAppAction.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'START'.
enum
{
  hqv_public_interface__action__MowerAppAction_Goal__START = 0
};

/// Constant 'PAUSE'.
enum
{
  hqv_public_interface__action__MowerAppAction_Goal__PAUSE = 1
};

/// Constant 'ABORT_START'.
enum
{
  hqv_public_interface__action__MowerAppAction_Goal__ABORT_START = 2
};

/// Constant 'CONFIRM_ERROR'.
enum
{
  hqv_public_interface__action__MowerAppAction_Goal__CONFIRM_ERROR = 3
};

// Struct defined in action/MowerAppAction in the package hqv_public_interface.
typedef struct hqv_public_interface__action__MowerAppAction_Goal
{
  uint8_t action;
} hqv_public_interface__action__MowerAppAction_Goal;

// Struct for a sequence of hqv_public_interface__action__MowerAppAction_Goal.
typedef struct hqv_public_interface__action__MowerAppAction_Goal__Sequence
{
  hqv_public_interface__action__MowerAppAction_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__MowerAppAction_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/MowerAppAction in the package hqv_public_interface.
typedef struct hqv_public_interface__action__MowerAppAction_Result
{
  bool result;
} hqv_public_interface__action__MowerAppAction_Result;

// Struct for a sequence of hqv_public_interface__action__MowerAppAction_Result.
typedef struct hqv_public_interface__action__MowerAppAction_Result__Sequence
{
  hqv_public_interface__action__MowerAppAction_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__MowerAppAction_Result__Sequence;


// Constants defined in the message

// Struct defined in action/MowerAppAction in the package hqv_public_interface.
typedef struct hqv_public_interface__action__MowerAppAction_Feedback
{
  uint8_t progress;
} hqv_public_interface__action__MowerAppAction_Feedback;

// Struct for a sequence of hqv_public_interface__action__MowerAppAction_Feedback.
typedef struct hqv_public_interface__action__MowerAppAction_Feedback__Sequence
{
  hqv_public_interface__action__MowerAppAction_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__MowerAppAction_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "hqv_public_interface/action/detail/mower_app_action__struct.h"

// Struct defined in action/MowerAppAction in the package hqv_public_interface.
typedef struct hqv_public_interface__action__MowerAppAction_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  hqv_public_interface__action__MowerAppAction_Goal goal;
} hqv_public_interface__action__MowerAppAction_SendGoal_Request;

// Struct for a sequence of hqv_public_interface__action__MowerAppAction_SendGoal_Request.
typedef struct hqv_public_interface__action__MowerAppAction_SendGoal_Request__Sequence
{
  hqv_public_interface__action__MowerAppAction_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__MowerAppAction_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/MowerAppAction in the package hqv_public_interface.
typedef struct hqv_public_interface__action__MowerAppAction_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} hqv_public_interface__action__MowerAppAction_SendGoal_Response;

// Struct for a sequence of hqv_public_interface__action__MowerAppAction_SendGoal_Response.
typedef struct hqv_public_interface__action__MowerAppAction_SendGoal_Response__Sequence
{
  hqv_public_interface__action__MowerAppAction_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__MowerAppAction_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/MowerAppAction in the package hqv_public_interface.
typedef struct hqv_public_interface__action__MowerAppAction_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} hqv_public_interface__action__MowerAppAction_GetResult_Request;

// Struct for a sequence of hqv_public_interface__action__MowerAppAction_GetResult_Request.
typedef struct hqv_public_interface__action__MowerAppAction_GetResult_Request__Sequence
{
  hqv_public_interface__action__MowerAppAction_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__MowerAppAction_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"

// Struct defined in action/MowerAppAction in the package hqv_public_interface.
typedef struct hqv_public_interface__action__MowerAppAction_GetResult_Response
{
  int8_t status;
  hqv_public_interface__action__MowerAppAction_Result result;
} hqv_public_interface__action__MowerAppAction_GetResult_Response;

// Struct for a sequence of hqv_public_interface__action__MowerAppAction_GetResult_Response.
typedef struct hqv_public_interface__action__MowerAppAction_GetResult_Response__Sequence
{
  hqv_public_interface__action__MowerAppAction_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__MowerAppAction_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__struct.h"

// Struct defined in action/MowerAppAction in the package hqv_public_interface.
typedef struct hqv_public_interface__action__MowerAppAction_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  hqv_public_interface__action__MowerAppAction_Feedback feedback;
} hqv_public_interface__action__MowerAppAction_FeedbackMessage;

// Struct for a sequence of hqv_public_interface__action__MowerAppAction_FeedbackMessage.
typedef struct hqv_public_interface__action__MowerAppAction_FeedbackMessage__Sequence
{
  hqv_public_interface__action__MowerAppAction_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__MowerAppAction_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__STRUCT_H_
