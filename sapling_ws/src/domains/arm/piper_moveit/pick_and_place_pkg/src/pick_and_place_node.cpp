#include "pick_and_place_node.hpp"

PickAndPlace::PickAndPlace(rclcpp::Node::SharedPtr node, const std::string& group_name) : node_(node)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

    // Your teammate's exact settings
    move_group_->setPlanningTime(2.0);
    move_group_->setNumPlanningAttempts(100);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
    move_group_->setEndEffectorLink("grasp_tcp"); 
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setGoalPositionTolerance(0.001);
    move_group_->setGoalOrientationTolerance(0.01); //radians

    // Initialize the Service Server
    service_ = node_->create_service<piper_msgs::srv::MoveToPose>(
        "move_to_pose",
        std::bind(&PickAndPlace::handle_service_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(node_->get_logger(), "PickAndPlace node initialized. Service '/move_to_pose' is ready.");
}

void PickAndPlace::handle_service_request(
    const std::shared_ptr<piper_msgs::srv::MoveToPose::Request> request,
    std::shared_ptr<piper_msgs::srv::MoveToPose::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "Pose received from Master Node. Planning...");
    // Call the teammate's exact move_to_pose function and return the success result
    response->success = move_to_pose(request->pose);
}

// Your teammate's exact movement logic
bool PickAndPlace::move_to_pose(const geometry_msgs::msg::Pose& target_pose)
{

    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();

    move_group_->setStartStateToCurrentState();

    move_group_->setPoseTarget(target_pose);

    rclcpp::sleep_for(std::chrono::milliseconds(200));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "Plan succeeded. Executing...");
        move_group_->move();
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    }

    return success;
}

bool PickAndPlace::return_home()
{
    move_group_->setNamedTarget("zero");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_->execute(plan);
        return true;
    }
    return false;
}
