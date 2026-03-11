#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "piper_msgs/srv/move_to_home.hpp"

class HomeControl
{
public:
    HomeControl(rclcpp::Node::SharedPtr node, const std::string& group_name);

    bool return_home();

private:
    void handle_service_request(
        const std::shared_ptr<piper_msgs::srv::MoveToHome::Request> request,
        std::shared_ptr<piper_msgs::srv::MoveToHome::Response> response);

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Service<piper_msgs::srv::MoveToHome>::SharedPtr service_;
};
