#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "piper_msgs/srv/set_grip_width.hpp"

class GripperControl
{
public:
    GripperControl(rclcpp::Node::SharedPtr node, const std::string& group_name);

    // 1 (true) = open, 0 (false) = close
    bool set_gripper_state(bool state);

private:
    void handle_service_request(
        const std::shared_ptr<piper_msgs::srv::SetGripWidth::Request> request,
        std::shared_ptr<piper_msgs::srv::SetGripWidth::Response> response);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Service<piper_msgs::srv::SetGripWidth>::SharedPtr service_;
};
