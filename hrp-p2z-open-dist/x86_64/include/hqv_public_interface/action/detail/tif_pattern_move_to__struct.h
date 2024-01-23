// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hqv_public_interface:action/TifPatternMoveTo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__STRUCT_H_
#define HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in action/TifPatternMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__action__TifPatternMoveTo_Goal
{
  float x;
  float y;
  float spd;
  float line_a_x;
  float line_a_y;
  float line_b_x;
  float line_b_y;
  float path_offset;
  bool randomize_path_offset;
  bool merge_at_end;
  bool follow_path_backwards;
  uint8_t finish_action;
} hqv_public_interface__action__TifPatternMoveTo_Goal;

// Struct for a sequence of hqv_public_interface__action__TifPatternMoveTo_Goal.
typedef struct hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence
{
  hqv_public_interface__action__TifPatternMoveTo_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__TifPatternMoveTo_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/TifPatternMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__action__TifPatternMoveTo_Result
{
  uint8_t result;
} hqv_public_interface__action__TifPatternMoveTo_Result;

// Struct for a sequence of hqv_public_interface__action__TifPatternMoveTo_Result.
typedef struct hqv_public_interface__action__TifPatternMoveTo_Result__Sequence
{
  hqv_public_interface__action__TifPatternMoveTo_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__TifPatternMoveTo_Result__Sequence;


// Constants defined in the message

// Struct defined in action/TifPatternMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__action__TifPatternMoveTo_Feedback
{
  uint8_t prog;
} hqv_public_interface__action__TifPatternMoveTo_Feedback;

// Struct for a sequence of hqv_public_interface__action__TifPatternMoveTo_Feedback.
typedef struct hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence
{
  hqv_public_interface__action__TifPatternMoveTo_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__TifPatternMoveTo_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"

// Struct defined in action/TifPatternMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  hqv_public_interface__action__TifPatternMoveTo_Goal goal;
} hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request;

// Struct for a sequence of hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request.
typedef struct hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence
{
  hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__TifPatternMoveTo_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/TifPatternMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response;

// Struct for a sequence of hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response.
typedef struct hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence
{
  hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__TifPatternMoveTo_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/TifPatternMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__action__TifPatternMoveTo_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} hqv_public_interface__action__TifPatternMoveTo_GetResult_Request;

// Struct for a sequence of hqv_public_interface__action__TifPatternMoveTo_GetResult_Request.
typedef struct hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence
{
  hqv_public_interface__action__TifPatternMoveTo_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__TifPatternMoveTo_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"

// Struct defined in action/TifPatternMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__action__TifPatternMoveTo_GetResult_Response
{
  int8_t status;
  hqv_public_interface__action__TifPatternMoveTo_Result result;
} hqv_public_interface__action__TifPatternMoveTo_GetResult_Response;

// Struct for a sequence of hqv_public_interface__action__TifPatternMoveTo_GetResult_Response.
typedef struct hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence
{
  hqv_public_interface__action__TifPatternMoveTo_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__TifPatternMoveTo_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.h"

// Struct defined in action/TifPatternMoveTo in the package hqv_public_interface.
typedef struct hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  hqv_public_interface__action__TifPatternMoveTo_Feedback feedback;
} hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage;

// Struct for a sequence of hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage.
typedef struct hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence
{
  hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hqv_public_interface__action__TifPatternMoveTo_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__STRUCT_H_
