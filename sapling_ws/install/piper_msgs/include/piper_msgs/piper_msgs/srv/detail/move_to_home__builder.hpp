// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from piper_msgs:srv/MoveToHome.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__MOVE_TO_HOME__BUILDER_HPP_
#define PIPER_MSGS__SRV__DETAIL__MOVE_TO_HOME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "piper_msgs/srv/detail/move_to_home__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace piper_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::MoveToHome_Request>()
{
  return ::piper_msgs::srv::MoveToHome_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace piper_msgs


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_MoveToHome_Response_success
{
public:
  Init_MoveToHome_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::MoveToHome_Response success(::piper_msgs::srv::MoveToHome_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::MoveToHome_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::MoveToHome_Response>()
{
  return piper_msgs::srv::builder::Init_MoveToHome_Response_success();
}

}  // namespace piper_msgs

#endif  // PIPER_MSGS__SRV__DETAIL__MOVE_TO_HOME__BUILDER_HPP_
