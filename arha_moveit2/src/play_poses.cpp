#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

struct Pose {
    std::vector<double> left;
    std::vector<double> right;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("play_poses_node", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get joint states
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Define the joint names in the exact order they appear in the URDF/JointState
    std::vector<std::string> left_joints = {
        "ShoulderLU_joint", "ShoulderLD_joint", "ElbowLU_joint", 
        "ElbowLD_joint", "WristLU_joint", "WristLD_joint"
    };
    std::vector<std::string> right_joints = {
        "ShoulderRU_joint", "ShoulderRD_joint", "ElbowRU_joint", 
        "ElbowRD_joint", "WristRU_joint", "WristRD_joint"
    };

    std::vector<Pose> poses = {
        { // Pose 1
            {-1.5706,  0.0, -0.0001, -1.5706, -0.0001,  0.0},
            { 1.5706,  0.0,  0.0,     1.5704,  0.0,     0.0}
        },
        { // Pose 2 (formerly Pose 3)
            {-1.5706,  0.0,  0.0,     0.0,     1.5707,  0.0},
            { 1.5636,  0.0,  0.0380,  0.0,    -1.5704,  0.0}
        },
        { // Pose 3 (formerly Pose 4)
            {-1.0335,  0.0,  1.5645, -1.5704, -1.1306,  0.0},
            { 1.5704,  0.0, -1.5704,  1.5706, -1.5702, -0.0078}
        },
        { // Pose 4 (formerly Pose 5)
            {-1.5704,  0.0,  0.0075, -1.5706, -1.5704,  0.0},
            { 1.5704,  0.0,  0.0054,  0.0,    -1.5706, -0.0078}
        },
        { // Pose 5 (formerly Pose 6)
            { 0.0055,  0.3654,  1.5666, -1.5706, -1.5704,  0.0},
            { 0.0055, -0.3652, -1.5706,  1.5704,  0.0,    -0.0080}
        }
    };

    RCLCPP_INFO(node->get_logger(), "Connecting to MoveGroups...");
    moveit::planning_interface::MoveGroupInterface right_arm(node, "right_arm");
    moveit::planning_interface::MoveGroupInterface left_arm(node, "left_arm");

    // Optional: Set speed and acceleration scaling to ensure smooth, safe movement
    right_arm.setMaxVelocityScalingFactor(0.2);
    right_arm.setMaxAccelerationScalingFactor(0.2);
    left_arm.setMaxVelocityScalingFactor(0.2);
    left_arm.setMaxAccelerationScalingFactor(0.2);

    for (size_t i = 0; i < poses.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "=== Moving to Pose %zu ===", i + 1);

        const int MAX_RETRIES = 10;
        
        // --- Move Right Arm ---
        bool right_completed = false;
        for (int retry = 1; retry <= MAX_RETRIES; ++retry) {
            RCLCPP_INFO(node->get_logger(), "Planning right arm (Attempt %d)...", retry);
            right_arm.setJointValueTarget(right_joints, poses[i].right);
            
            moveit::planning_interface::MoveGroupInterface::Plan right_plan;
            if (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Executing right arm...");
                auto exec_result = right_arm.execute(right_plan);
                if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    right_completed = true;
                    break;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Right arm execution aborted! Retrying...");
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "Right arm planning failed! Retrying...");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        if (!right_completed) {
            RCLCPP_ERROR(node->get_logger(), "Right arm failed to reach pose %zu after %d attempts! Aborting sequence to prevent collisions.", i + 1, MAX_RETRIES);
            break;
        }

        // --- Move Left Arm ---
        bool left_completed = false;
        for (int retry = 1; retry <= MAX_RETRIES; ++retry) {
            RCLCPP_INFO(node->get_logger(), "Planning left arm (Attempt %d)...", retry);
            left_arm.setJointValueTarget(left_joints, poses[i].left);
            
            moveit::planning_interface::MoveGroupInterface::Plan left_plan;
            if (left_arm.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Executing left arm...");
                auto exec_result = left_arm.execute(left_plan);
                if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    left_completed = true;
                    break;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Left arm execution aborted! Retrying...");
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "Left arm planning failed! Retrying...");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        if (!left_completed) {
            RCLCPP_ERROR(node->get_logger(), "Left arm failed to reach pose %zu after %d attempts! Aborting sequence.", i + 1, MAX_RETRIES);
            break;
        }
        
        RCLCPP_INFO(node->get_logger(), "Waiting 1.0s before next pose.");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    RCLCPP_INFO(node->get_logger(), "Sequence complete.");
    rclcpp::shutdown();
    return 0;
}
