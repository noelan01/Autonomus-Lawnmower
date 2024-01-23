// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hqv_public_interface:action/MowerAppAction.idl
// generated code does not contain a copyright notice

#ifndef HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__BUILDER_HPP_
#define HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__BUILDER_HPP_

#include "hqv_public_interface/action/detail/mower_app_action__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_MowerAppAction_Goal_action
{
public:
  Init_MowerAppAction_Goal_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::action::MowerAppAction_Goal action(::hqv_public_interface::action::MowerAppAction_Goal::_action_type arg)
  {
    msg_.action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::MowerAppAction_Goal>()
{
  return hqv_public_interface::action::builder::Init_MowerAppAction_Goal_action();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_MowerAppAction_Result_result
{
public:
  Init_MowerAppAction_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::action::MowerAppAction_Result result(::hqv_public_interface::action::MowerAppAction_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::MowerAppAction_Result>()
{
  return hqv_public_interface::action::builder::Init_MowerAppAction_Result_result();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_MowerAppAction_Feedback_progress
{
public:
  Init_MowerAppAction_Feedback_progress()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::action::MowerAppAction_Feedback progress(::hqv_public_interface::action::MowerAppAction_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::MowerAppAction_Feedback>()
{
  return hqv_public_interface::action::builder::Init_MowerAppAction_Feedback_progress();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_MowerAppAction_SendGoal_Request_goal
{
public:
  explicit Init_MowerAppAction_SendGoal_Request_goal(::hqv_public_interface::action::MowerAppAction_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::MowerAppAction_SendGoal_Request goal(::hqv_public_interface::action::MowerAppAction_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_SendGoal_Request msg_;
};

class Init_MowerAppAction_SendGoal_Request_goal_id
{
public:
  Init_MowerAppAction_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerAppAction_SendGoal_Request_goal goal_id(::hqv_public_interface::action::MowerAppAction_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MowerAppAction_SendGoal_Request_goal(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::MowerAppAction_SendGoal_Request>()
{
  return hqv_public_interface::action::builder::Init_MowerAppAction_SendGoal_Request_goal_id();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_MowerAppAction_SendGoal_Response_stamp
{
public:
  explicit Init_MowerAppAction_SendGoal_Response_stamp(::hqv_public_interface::action::MowerAppAction_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::MowerAppAction_SendGoal_Response stamp(::hqv_public_interface::action::MowerAppAction_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_SendGoal_Response msg_;
};

class Init_MowerAppAction_SendGoal_Response_accepted
{
public:
  Init_MowerAppAction_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerAppAction_SendGoal_Response_stamp accepted(::hqv_public_interface::action::MowerAppAction_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MowerAppAction_SendGoal_Response_stamp(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::MowerAppAction_SendGoal_Response>()
{
  return hqv_public_interface::action::builder::Init_MowerAppAction_SendGoal_Response_accepted();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_MowerAppAction_GetResult_Request_goal_id
{
public:
  Init_MowerAppAction_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hqv_public_interface::action::MowerAppAction_GetResult_Request goal_id(::hqv_public_interface::action::MowerAppAction_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::MowerAppAction_GetResult_Request>()
{
  return hqv_public_interface::action::builder::Init_MowerAppAction_GetResult_Request_goal_id();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_MowerAppAction_GetResult_Response_result
{
public:
  explicit Init_MowerAppAction_GetResult_Response_result(::hqv_public_interface::action::MowerAppAction_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::MowerAppAction_GetResult_Response result(::hqv_public_interface::action::MowerAppAction_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_GetResult_Response msg_;
};

class Init_MowerAppAction_GetResult_Response_status
{
public:
  Init_MowerAppAction_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerAppAction_GetResult_Response_result status(::hqv_public_interface::action::MowerAppAction_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MowerAppAction_GetResult_Response_result(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::MowerAppAction_GetResult_Response>()
{
  return hqv_public_interface::action::builder::Init_MowerAppAction_GetResult_Response_status();
}

}  // namespace hqv_public_interface


namespace hqv_public_interface
{

namespace action
{

namespace builder
{

class Init_MowerAppAction_FeedbackMessage_feedback
{
public:
  explicit Init_MowerAppAction_FeedbackMessage_feedback(::hqv_public_interface::action::MowerAppAction_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::hqv_public_interface::action::MowerAppAction_FeedbackMessage feedback(::hqv_public_interface::action::MowerAppAction_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_FeedbackMessage msg_;
};

class Init_MowerAppAction_FeedbackMessage_goal_id
{
public:
  Init_MowerAppAction_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerAppAction_FeedbackMessage_feedback goal_id(::hqv_public_interface::action::MowerAppAction_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MowerAppAction_FeedbackMessage_feedback(msg_);
  }

private:
  ::hqv_public_interface::action::MowerAppAction_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hqv_public_interface::action::MowerAppAction_FeedbackMessage>()
{
  return hqv_public_interface::action::builder::Init_MowerAppAction_FeedbackMessage_goal_id();
}

}  // namespace hqv_public_interface

#endif  // HQV_PUBLIC_INTERFACE__ACTION__DETAIL__MOWER_APP_ACTION__BUILDER_HPP_
