#pragma once

#include <memory>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "piper_msgs/srv/move_to_pose.hpp"

class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node, const std::string& group_name);

    bool move_to_pose(const geometry_msgs::msg::Pose& target_pose);
    bool return_home();

private:
    // This is the function that triggers when the Python node calls the service
    void handle_service_request(
        const std::shared_ptr<piper_msgs::srv::MoveToPose::Request> request,
        std::shared_ptr<piper_msgs::srv::MoveToPose::Response> response);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Service<piper_msgs::srv::MoveToPose>::SharedPtr service_;
};
