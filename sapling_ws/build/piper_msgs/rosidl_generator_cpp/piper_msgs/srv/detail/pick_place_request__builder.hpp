// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from piper_msgs:srv/PickPlaceRequest.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__PICK_PLACE_REQUEST__BUILDER_HPP_
#define PIPER_MSGS__SRV__DETAIL__PICK_PLACE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "piper_msgs/srv/detail/pick_place_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_PickPlaceRequest_Request_pose
{
public:
  Init_PickPlaceRequest_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::PickPlaceRequest_Request pose(::piper_msgs::srv::PickPlaceRequest_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::PickPlaceRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::PickPlaceRequest_Request>()
{
  return piper_msgs::srv::builder::Init_PickPlaceRequest_Request_pose();
}

}  // namespace piper_msgs


namespace piper_msgs
{

namespace srv
{

namespace builder
{

class Init_PickPlaceRequest_Response_success
{
public:
  Init_PickPlaceRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::piper_msgs::srv::PickPlaceRequest_Response success(::piper_msgs::srv::PickPlaceRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::piper_msgs::srv::PickPlaceRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::piper_msgs::srv::PickPlaceRequest_Response>()
{
  return piper_msgs::srv::builder::Init_PickPlaceRequest_Response_success();
}

}  // namespace piper_msgs

#endif  // PIPER_MSGS__SRV__DETAIL__PICK_PLACE_REQUEST__BUILDER_HPP_
