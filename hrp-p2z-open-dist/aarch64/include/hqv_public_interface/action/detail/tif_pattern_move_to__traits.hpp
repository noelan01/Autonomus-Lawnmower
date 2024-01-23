// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:action/TifPatternMoveTo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__TRAITS_HPP_

#include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::TifPatternMoveTo_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: spd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spd: ";
    value_to_yaml(msg.spd, out);
    out << "\n";
  }

  // member: line_a_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "line_a_x: ";
    value_to_yaml(msg.line_a_x, out);
    out << "\n";
  }

  // member: line_a_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "line_a_y: ";
    value_to_yaml(msg.line_a_y, out);
    out << "\n";
  }

  // member: line_b_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "line_b_x: ";
    value_to_yaml(msg.line_b_x, out);
    out << "\n";
  }

  // member: line_b_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "line_b_y: ";
    value_to_yaml(msg.line_b_y, out);
    out << "\n";
  }

  // member: path_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path_offset: ";
    value_to_yaml(msg.path_offset, out);
    out << "\n";
  }

  // member: randomize_path_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "randomize_path_offset: ";
    value_to_yaml(msg.randomize_path_offset, out);
    out << "\n";
  }

  // member: merge_at_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "merge_at_end: ";
    value_to_yaml(msg.merge_at_end, out);
    out << "\n";
  }

  // member: follow_path_backwards
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "follow_path_backwards: ";
    value_to_yaml(msg.follow_path_backwards, out);
    out << "\n";
  }

  // member: finish_action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "finish_action: ";
    value_to_yaml(msg.finish_action, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_Goal>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_Goal";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_Goal>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_Goal";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::action::TifPatternMoveTo_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::TifPatternMoveTo_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::TifPatternMoveTo_Result & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_Result>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_Result";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_Result>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_Result";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::action::TifPatternMoveTo_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::TifPatternMoveTo_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: prog
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prog: ";
    value_to_yaml(msg.prog, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::TifPatternMoveTo_Feedback & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_Feedback>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_Feedback";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_Feedback>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_Feedback";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::action::TifPatternMoveTo_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "hqv_public_interface/action/detail/tif_pattern_move_to__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_SendGoal_Request";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_SendGoal_Response";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_SendGoal>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_SendGoal";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_SendGoal>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_SendGoal";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>::value &&
    has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>::value &&
    has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<hqv_public_interface::action::TifPatternMoveTo_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::TifPatternMoveTo_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::TifPatternMoveTo_GetResult_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_GetResult_Request";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_GetResult_Request";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::TifPatternMoveTo_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::TifPatternMoveTo_GetResult_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_GetResult_Response";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_GetResult_Response";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_Result>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_Result>::value> {};

template<>
struct is_message<hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_GetResult>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_GetResult";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_GetResult>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_GetResult";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>::value &&
    has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>::value &&
    has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>::value
  >
{
};

template<>
struct is_service<hqv_public_interface::action::TifPatternMoveTo_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "hqv_public_interface/action/detail/tif_pattern_move_to__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage>()
{
  return "hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage";
}

template<>
inline const char * name<hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage>()
{
  return "hqv_public_interface/action/TifPatternMoveTo_FeedbackMessage";
}

template<>
struct has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<hqv_public_interface::action::TifPatternMoveTo_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<hqv_public_interface::action::TifPatternMoveTo_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<hqv_public_interface::action::TifPatternMoveTo>
  : std::true_type
{
};

template<>
struct is_action_goal<hqv_public_interface::action::TifPatternMoveTo_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<hqv_public_interface::action::TifPatternMoveTo_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<hqv_public_interface::action::TifPatternMoveTo_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__TRAITS_HPP_
