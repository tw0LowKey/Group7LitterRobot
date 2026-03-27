#include "pick_and_place_node.hpp"

PickAndPlace::PickAndPlace(rclcpp::Node::SharedPtr node, const std::string& group_name) : node_(node)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);

    move_group_->setPlanningTime(1.0);
    move_group_->setNumPlanningAttempts(100);
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
    move_group_->setEndEffectorLink("grasp_tcp"); 
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setGoalPositionTolerance(0.001);
    move_group_->setGoalOrientationTolerance(0.01);

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
    response->success = move_to_pose(request->pose);
}

bool PickAndPlace::move_to_pose(const geometry_msgs::msg::Pose& target_pose)
{
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();

    // Compute pre-approach pose (10cm back along tool Z)
    tf2::Quaternion q(
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);

    tf2::Matrix3x3 rot(q);
    tf2::Vector3 approach_dir = rot.getColumn(2);

    geometry_msgs::msg::Pose pre_pose = target_pose;
    double offset = 0.10;
    pre_pose.position.x -= offset * approach_dir.x();
    pre_pose.position.y -= offset * approach_dir.y();
    pre_pose.position.z -= offset * approach_dir.z();

    // -------------------------------
    // PHASE 1: Move to pre-approach pose
    // -------------------------------
    move_group_->setPoseTarget(pre_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (move_group_->plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success1)
    {
        RCLCPP_ERROR(node_->get_logger(), "Pre-approach planning failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Phase 1 plan succeeded. Executing...");
    move_group_->execute(plan1);

    // -------------------------------
    // PHASE 2: Final approach to target pose
    // -------------------------------
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();

    move_group_->setPoseTarget(target_pose);
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (move_group_->plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success2)
    {
        RCLCPP_ERROR(node_->get_logger(), "Final approach planning failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Phase 2 plan succeeded. Executing...");
    move_group_->execute(plan2);

    return true;
}