// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:action/TifPatternMoveTo.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__BUILDER_HPP_

#include "hqv_public_interface/action/detail/tif_pattern_move_to__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_TifPatternMoveTo_Goal_finish_action
{
public:
  explicit Init_TifPatternMoveTo_Goal_finish_action(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::TifPatternMoveTo_Goal finish_action(::hqv_public_interface::action::TifPatternMoveTo_Goal::_finish_action_type arg)
  {
    msg_.finish_action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_follow_path_backwards
{
public:
  explicit Init_TifPatternMoveTo_Goal_follow_path_backwards(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_finish_action follow_path_backwards(::hqv_public_interface::action::TifPatternMoveTo_Goal::_follow_path_backwards_type arg)
  {
    msg_.follow_path_backwards = std::move(arg);
    return Init_TifPatternMoveTo_Goal_finish_action(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_merge_at_end
{
public:
  explicit Init_TifPatternMoveTo_Goal_merge_at_end(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_follow_path_backwards merge_at_end(::hqv_public_interface::action::TifPatternMoveTo_Goal::_merge_at_end_type arg)
  {
    msg_.merge_at_end = std::move(arg);
    return Init_TifPatternMoveTo_Goal_follow_path_backwards(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_randomize_path_offset
{
public:
  explicit Init_TifPatternMoveTo_Goal_randomize_path_offset(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_merge_at_end randomize_path_offset(::hqv_public_interface::action::TifPatternMoveTo_Goal::_randomize_path_offset_type arg)
  {
    msg_.randomize_path_offset = std::move(arg);
    return Init_TifPatternMoveTo_Goal_merge_at_end(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_path_offset
{
public:
  explicit Init_TifPatternMoveTo_Goal_path_offset(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_randomize_path_offset path_offset(::hqv_public_interface::action::TifPatternMoveTo_Goal::_path_offset_type arg)
  {
    msg_.path_offset = std::move(arg);
    return Init_TifPatternMoveTo_Goal_randomize_path_offset(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_line_b_y
{
public:
  explicit Init_TifPatternMoveTo_Goal_line_b_y(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_path_offset line_b_y(::hqv_public_interface::action::TifPatternMoveTo_Goal::_line_b_y_type arg)
  {
    msg_.line_b_y = std::move(arg);
    return Init_TifPatternMoveTo_Goal_path_offset(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_line_b_x
{
public:
  explicit Init_TifPatternMoveTo_Goal_line_b_x(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_line_b_y line_b_x(::hqv_public_interface::action::TifPatternMoveTo_Goal::_line_b_x_type arg)
  {
    msg_.line_b_x = std::move(arg);
    return Init_TifPatternMoveTo_Goal_line_b_y(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_line_a_y
{
public:
  explicit Init_TifPatternMoveTo_Goal_line_a_y(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_line_b_x line_a_y(::hqv_public_interface::action::TifPatternMoveTo_Goal::_line_a_y_type arg)
  {
    msg_.line_a_y = std::move(arg);
    return Init_TifPatternMoveTo_Goal_line_b_x(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_line_a_x
{
public:
  explicit Init_TifPatternMoveTo_Goal_line_a_x(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_line_a_y line_a_x(::hqv_public_interface::action::TifPatternMoveTo_Goal::_line_a_x_type arg)
  {
    msg_.line_a_x = std::move(arg);
    return Init_TifPatternMoveTo_Goal_line_a_y(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_spd
{
public:
  explicit Init_TifPatternMoveTo_Goal_spd(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_line_a_x spd(::hqv_public_interface::action::TifPatternMoveTo_Goal::_spd_type arg)
  {
    msg_.spd = std::move(arg);
    return Init_TifPatternMoveTo_Goal_line_a_x(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_y
{
public:
  explicit Init_TifPatternMoveTo_Goal_y(::hqv_public_interface::action::TifPatternMoveTo_Goal & msg)
  : msg_(msg)
  {}
  Init_TifPatternMoveTo_Goal_spd y(::hqv_public_interface::action::TifPatternMoveTo_Goal::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_TifPatternMoveTo_Goal_spd(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

class Init_TifPatternMoveTo_Goal_x
{
public:
  Init_TifPatternMoveTo_Goal_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TifPatternMoveTo_Goal_y x(::hqv_public_interface::action::TifPatternMoveTo_Goal::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_TifPatternMoveTo_Goal_y(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::TifPatternMoveTo_Goal>()
{
  return hqv_public_interface::action::builder::Init_TifPatternMoveTo_Goal_x();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_TifPatternMoveTo_Result_result
{
public:
  Init_TifPatternMoveTo_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::action::TifPatternMoveTo_Result result(::hqv_public_interface::action::TifPatternMoveTo_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::TifPatternMoveTo_Result>()
{
  return hqv_public_interface::action::builder::Init_TifPatternMoveTo_Result_result();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_TifPatternMoveTo_Feedback_prog
{
public:
  Init_TifPatternMoveTo_Feedback_prog()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::action::TifPatternMoveTo_Feedback prog(::hqv_public_interface::action::TifPatternMoveTo_Feedback::_prog_type arg)
  {
    msg_.prog = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::TifPatternMoveTo_Feedback>()
{
  return hqv_public_interface::action::builder::Init_TifPatternMoveTo_Feedback_prog();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_TifPatternMoveTo_SendGoal_Request_goal
{
public:
  explicit Init_TifPatternMoveTo_SendGoal_Request_goal(::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request goal(::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request msg_;
};

class Init_TifPatternMoveTo_SendGoal_Request_goal_id
{
public:
  Init_TifPatternMoveTo_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TifPatternMoveTo_SendGoal_Request_goal goal_id(::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_TifPatternMoveTo_SendGoal_Request_goal(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Request>()
{
  return hqv_public_interface::action::builder::Init_TifPatternMoveTo_SendGoal_Request_goal_id();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_TifPatternMoveTo_SendGoal_Response_stamp
{
public:
  explicit Init_TifPatternMoveTo_SendGoal_Response_stamp(::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response stamp(::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response msg_;
};

class Init_TifPatternMoveTo_SendGoal_Response_accepted
{
public:
  Init_TifPatternMoveTo_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TifPatternMoveTo_SendGoal_Response_stamp accepted(::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_TifPatternMoveTo_SendGoal_Response_stamp(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::TifPatternMoveTo_SendGoal_Response>()
{
  return hqv_public_interface::action::builder::Init_TifPatternMoveTo_SendGoal_Response_accepted();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_TifPatternMoveTo_GetResult_Request_goal_id
{
public:
  Init_TifPatternMoveTo_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::action::TifPatternMoveTo_GetResult_Request goal_id(::hqv_public_interface::action::TifPatternMoveTo_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::TifPatternMoveTo_GetResult_Request>()
{
  return hqv_public_interface::action::builder::Init_TifPatternMoveTo_GetResult_Request_goal_id();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_TifPatternMoveTo_GetResult_Response_result
{
public:
  explicit Init_TifPatternMoveTo_GetResult_Response_result(::hqv_public_interface::action::TifPatternMoveTo_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::TifPatternMoveTo_GetResult_Response result(::hqv_public_interface::action::TifPatternMoveTo_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_GetResult_Response msg_;
};

class Init_TifPatternMoveTo_GetResult_Response_status
{
public:
  Init_TifPatternMoveTo_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TifPatternMoveTo_GetResult_Response_result status(::hqv_public_interface::action::TifPatternMoveTo_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_TifPatternMoveTo_GetResult_Response_result(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::TifPatternMoveTo_GetResult_Response>()
{
  return hqv_public_interface::action::builder::Init_TifPatternMoveTo_GetResult_Response_status();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_TifPatternMoveTo_FeedbackMessage_feedback
{
public:
  explicit Init_TifPatternMoveTo_FeedbackMessage_feedback(::hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage feedback(::hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage msg_;
};

class Init_TifPatternMoveTo_FeedbackMessage_goal_id
{
public:
  Init_TifPatternMoveTo_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TifPatternMoveTo_FeedbackMessage_feedback goal_id(::hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_TifPatternMoveTo_FeedbackMessage_feedback(msg_);
  }

private:
  ::hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::TifPatternMoveTo_FeedbackMessage>()
{
  return hqv_public_interface::action::builder::Init_TifPatternMoveTo_FeedbackMessage_goal_id();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__ACTION__DETAIL__TIF_PATTERN_MOVE_TO__BUILDER_HPP_
