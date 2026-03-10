#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <vector>
#include <string>

struct Pose {
    std::vector<double> left;
    std::vector<double> right;
};

moveit_msgs::msg::Constraints createJointConstraints(const std::vector<std::string>& joint_names, const Pose& pose) {
    moveit_msgs::msg::Constraints constraints;
    for (size_t i = 0; i < 6; ++i) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = joint_names[i]; // Left
        jc.position = pose.left[i];
        jc.tolerance_above = 0.001;
        jc.tolerance_below = 0.001;
        jc.weight = 1.0;
        constraints.joint_constraints.push_back(jc);
    }
    for (size_t i = 0; i < 6; ++i) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = joint_names[6 + i]; // Right
        jc.position = pose.right[i];
        jc.tolerance_above = 0.001;
        jc.tolerance_below = 0.001;
        jc.weight = 1.0;
        constraints.joint_constraints.push_back(jc);
    }
    return constraints;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("play_poses_pilz", node_options);
    
    // We spin up a SingleThreadedExecutor so the MoveGroupInterface and action client can operate
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    std::vector<std::string> joint_names = {
        "ShoulderLU_joint", "ShoulderLD_joint", "ElbowLU_joint", 
        "ElbowLD_joint", "WristLU_joint", "WristLD_joint",
        "ShoulderRU_joint", "ShoulderRD_joint", "ElbowRU_joint", 
        "ElbowRD_joint", "WristRU_joint", "WristRD_joint"
    };

    std::vector<Pose> poses = {
        { // Pose 1
            {-1.570,  0.001, -0.001, -1.570, -0.001,  0.0},
            { 1.570, -0.001,  0.0,    1.570,  0.0,    0.0}
        },
        { // Pose 2 (formerly Pose 3)
            {-1.570,  0.001,  0.0,   -0.001,  1.570,  0.0},
            { 1.564, -0.001,  0.038,  0.001, -1.570,  0.0}
        },
        { // Pose 3 (formerly Pose 4)
            {-1.033,  0.001,  1.565, -1.570, -1.130,  0.0},
            { 1.570, -0.001, -1.570,  1.570, -1.570, -0.008}
        },
        { // Pose 4 (formerly Pose 5)
            {-1.570,  0.001,  0.008, -1.570, -1.570,  0.0},
            { 1.570, -0.001,  0.005,  0.001, -1.570, -0.008}
        },
        { // Pose 5 (formerly Pose 6)
            { 0.006,  0.365,  1.567, -1.570, -1.570,  0.0},
            { 0.006, -0.365, -1.570,  1.570,  0.0,   -0.008}
        }
    };

    RCLCPP_INFO(node->get_logger(), "Connecting to MoveGroup 'both_arms'...");
    moveit::planning_interface::MoveGroupInterface both_arms(node, "both_arms");
    
    using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
    auto action_client = rclcpp_action::create_client<MoveGroupSequence>(node, "sequence_move_group");
    
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available!");
        rclcpp::shutdown();
        return 1;
    }

    MoveGroupSequence::Goal goal;
    
    // We want the action server to plan and execute
    goal.planning_options.plan_only = false;
    goal.planning_options.look_around = false;

    // Create a series of Pilz Point-to-Point sequence items
    RCLCPP_INFO(node->get_logger(), "Constructing Pilz Sequence...");
    for (size_t i = 0; i < poses.size(); ++i) {
        moveit_msgs::msg::MotionSequenceItem item;
        item.req.group_name = "both_arms";
        item.req.pipeline_id = "pilz_industrial_motion_planner";
        item.req.planner_id = "PTP";
        item.req.workspace_parameters.header.frame_id = both_arms.getPlanningFrame();
        item.req.allowed_planning_time = 5.0;
        item.req.max_velocity_scaling_factor = 0.2;
        item.req.max_acceleration_scaling_factor = 0.2;
        
        // Pilz sequence planning requires the first item to have an explicit start state.
        // We set it to the current state of the robot.
        if (i == 0) {
            moveit::core::RobotStatePtr current_state = both_arms.getCurrentState();
            if (current_state) {
                moveit::core::robotStateToRobotStateMsg(*current_state, item.req.start_state);
            } else {
                RCLCPP_WARN(node->get_logger(), "Could not read current state, using empty start state.");
                item.req.start_state.is_diff = true;
            }
        }

        item.req.goal_constraints.push_back(createJointConstraints(joint_names, poses[i]));
        
        // This is the key to preventing the robot from stopping:
        // Set a blend radius for all intermediate poses so it curves through them smoothly!
        if (i < poses.size() - 1) {
            item.blend_radius = 0.1; 
        } else {
            item.blend_radius = 0.0;
        }
        
        goal.request.items.push_back(item);
    }

    RCLCPP_INFO(node->get_logger(), "Sending Pilz Sequence Goal to Action Server...");
    auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
    
    auto goal_handle_future = action_client->async_send_goal(goal, send_goal_options);
    
    // The background thread is spinning the node, so we can just wait for the future
    if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        RCLCPP_ERROR(node->get_logger(), "Timeout waiting for goal acceptance");
        rclcpp::shutdown();
        return 1;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Sequence Goal was rejected by server");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Goal accepted! Waiting for execution to complete...");
    auto result_future = action_client->async_get_result(goal_handle);
    result_future.wait();
    
    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "Pilz sequence finished executing cleanly!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Pilz sequence execution failed!");
    }

    rclcpp::shutdown();
    return 0;
}
