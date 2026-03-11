// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from piper_msgs:srv/SetGripWidth.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__BUILDER_HPP_
#define PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "piper_msgs/srv/detail/set_grip_width__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_SetGripWidth_Request_state
{
public:
  Init_SetGripWidth_Request_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::SetGripWidth_Request state(::piper_msgs::srv::SetGripWidth_Request::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::SetGripWidth_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::SetGripWidth_Request>()
{
  return piper_msgs::srv::builder::Init_SetGripWidth_Request_state();
}

}  // namespace piper_msgs


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_SetGripWidth_Response_success
{
public:
  Init_SetGripWidth_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::SetGripWidth_Response success(::piper_msgs::srv::SetGripWidth_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::SetGripWidth_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::SetGripWidth_Response>()
{
  return piper_msgs::srv::builder::Init_SetGripWidth_Response_success();
}

}  // namespace piper_msgs

#endif  // PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__BUILDER_HPP_
