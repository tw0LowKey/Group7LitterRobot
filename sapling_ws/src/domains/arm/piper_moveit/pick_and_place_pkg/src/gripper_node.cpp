#include "gripper_node.hpp"

GripperControl::GripperControl(rclcpp::Node::SharedPtr node, const std::string& group_name) : node_(node)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    service_ = node_->create_service<piper_msgs::srv::SetGripWidth>(
        "set_grip_width",
        std::bind(&GripperControl::handle_service_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(node_->get_logger(), "Gripper node initialized. Service '/set_grip_width' is ready.");
}

void GripperControl::handle_service_request(
    const std::shared_ptr<piper_msgs::srv::SetGripWidth::Request> request,
    std::shared_ptr<piper_msgs::srv::SetGripWidth::Response> response)
{
    // Print what we received (Translating the boolean to text for the console)
    RCLCPP_INFO(node_->get_logger(), "Gripper command received: %s", request->state ? "OPEN (True)" : "CLOSE (False)");
    
    // Pass the boolean to our execution function
    response->success = set_gripper_state(request->state);
}

bool GripperControl::set_gripper_state(bool state)
{
    // Translate the boolean (1/0) back into the named targets ("open"/"close") MoveIt needs
    std::string target_name = state ? "open" : "close";
    
    move_group_->setNamedTarget(target_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "Gripper plan succeeded. Executing %s...", target_name.c_str());
        move_group_->move();
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Gripper planning failed for state: '%s'. Does this named target exist?", target_name.c_str());
    }

    return success;
}
