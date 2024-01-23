// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hqv_public_interface:action/MowerAppAction.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__TRAITS_HPP_
#define HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__TRAITS_HPP_

#include "hqv_public_interface/action/detail/mower_app_action__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::MowerAppAction_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    value_to_yaml(msg.action, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::MowerAppAction_Goal & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_Goal>()
{
  return "hqv_public_interface::action::MowerAppAction_Goal";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_Goal>()
{
  return "hqv_public_interface/action/MowerAppAction_Goal";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::action::MowerAppAction_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::MowerAppAction_Result & msg,
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

inline std::string to_yaml(const hqv_public_interface::action::MowerAppAction_Result & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_Result>()
{
  return "hqv_public_interface::action::MowerAppAction_Result";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_Result>()
{
  return "hqv_public_interface/action/MowerAppAction_Result";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::action::MowerAppAction_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::MowerAppAction_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress: ";
    value_to_yaml(msg.progress, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const hqv_public_interface::action::MowerAppAction_Feedback & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_Feedback>()
{
  return "hqv_public_interface::action::MowerAppAction_Feedback";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_Feedback>()
{
  return "hqv_public_interface/action/MowerAppAction_Feedback";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hqv_public_interface::action::MowerAppAction_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "hqv_public_interface/action/detail/mower_app_action__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::MowerAppAction_SendGoal_Request & msg,
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

inline std::string to_yaml(const hqv_public_interface::action::MowerAppAction_SendGoal_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_SendGoal_Request>()
{
  return "hqv_public_interface::action::MowerAppAction_SendGoal_Request";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_SendGoal_Request>()
{
  return "hqv_public_interface/action/MowerAppAction_SendGoal_Request";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<hqv_public_interface::action::MowerAppAction_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<hqv_public_interface::action::MowerAppAction_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hqv_public_interface::action::MowerAppAction_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::MowerAppAction_SendGoal_Response & msg,
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

inline std::string to_yaml(const hqv_public_interface::action::MowerAppAction_SendGoal_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_SendGoal_Response>()
{
  return "hqv_public_interface::action::MowerAppAction_SendGoal_Response";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_SendGoal_Response>()
{
  return "hqv_public_interface/action/MowerAppAction_SendGoal_Response";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<hqv_public_interface::action::MowerAppAction_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_SendGoal>()
{
  return "hqv_public_interface::action::MowerAppAction_SendGoal";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_SendGoal>()
{
  return "hqv_public_interface/action/MowerAppAction_SendGoal";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<hqv_public_interface::action::MowerAppAction_SendGoal_Request>::value &&
    has_fixed_size<hqv_public_interface::action::MowerAppAction_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<hqv_public_interface::action::MowerAppAction_SendGoal_Request>::value &&
    has_bounded_size<hqv_public_interface::action::MowerAppAction_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<hqv_public_interface::action::MowerAppAction_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<hqv_public_interface::action::MowerAppAction_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hqv_public_interface::action::MowerAppAction_SendGoal_Response>
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
  const hqv_public_interface::action::MowerAppAction_GetResult_Request & msg,
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

inline std::string to_yaml(const hqv_public_interface::action::MowerAppAction_GetResult_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_GetResult_Request>()
{
  return "hqv_public_interface::action::MowerAppAction_GetResult_Request";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_GetResult_Request>()
{
  return "hqv_public_interface/action/MowerAppAction_GetResult_Request";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hqv_public_interface::action::MowerAppAction_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "hqv_public_interface/action/detail/mower_app_action__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::MowerAppAction_GetResult_Response & msg,
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

inline std::string to_yaml(const hqv_public_interface::action::MowerAppAction_GetResult_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_GetResult_Response>()
{
  return "hqv_public_interface::action::MowerAppAction_GetResult_Response";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_GetResult_Response>()
{
  return "hqv_public_interface/action/MowerAppAction_GetResult_Response";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<hqv_public_interface::action::MowerAppAction_Result>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<hqv_public_interface::action::MowerAppAction_Result>::value> {};

template<>
struct is_message<hqv_public_interface::action::MowerAppAction_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_GetResult>()
{
  return "hqv_public_interface::action::MowerAppAction_GetResult";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_GetResult>()
{
  return "hqv_public_interface/action/MowerAppAction_GetResult";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<hqv_public_interface::action::MowerAppAction_GetResult_Request>::value &&
    has_fixed_size<hqv_public_interface::action::MowerAppAction_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<hqv_public_interface::action::MowerAppAction_GetResult_Request>::value &&
    has_bounded_size<hqv_public_interface::action::MowerAppAction_GetResult_Response>::value
  >
{
};

template<>
struct is_service<hqv_public_interface::action::MowerAppAction_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<hqv_public_interface::action::MowerAppAction_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hqv_public_interface::action::MowerAppAction_GetResult_Response>
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
// #include "hqv_public_interface/action/detail/mower_app_action__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const hqv_public_interface::action::MowerAppAction_FeedbackMessage & msg,
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

inline std::string to_yaml(const hqv_public_interface::action::MowerAppAction_FeedbackMessage & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<hqv_public_interface::action::MowerAppAction_FeedbackMessage>()
{
  return "hqv_public_interface::action::MowerAppAction_FeedbackMessage";
}

template<>
inline const char * name<hqv_public_interface::action::MowerAppAction_FeedbackMessage>()
{
  return "hqv_public_interface/action/MowerAppAction_FeedbackMessage";
}

template<>
struct has_fixed_size<hqv_public_interface::action::MowerAppAction_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<hqv_public_interface::action::MowerAppAction_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hqv_public_interface::action::MowerAppAction_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<hqv_public_interface::action::MowerAppAction_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hqv_public_interface::action::MowerAppAction_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<hqv_public_interface::action::MowerAppAction>
  : std::true_type
{
};

template<>
struct is_action_goal<hqv_public_interface::action::MowerAppAction_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<hqv_public_interface::action::MowerAppAction_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<hqv_public_interface::action::MowerAppAction_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__TRAITS_HPP_
