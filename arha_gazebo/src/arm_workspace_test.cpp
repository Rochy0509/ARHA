#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>

class ArmWorkspaceTest : public rclcpp::Node
{
public:
  ArmWorkspaceTest() : Node("arm_workspace_test")
  {
    // Wait for MoveIt to be ready
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    try {
      left_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "left_arm");
      right_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "right_arm");
      
      RCLCPP_INFO(get_logger(), "MoveIt initialized - starting workspace tests");
      
      // Test timer
      timer_ = create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&ArmWorkspaceTest::testPoses, this));
        
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize: %s", e.what());
    }
  }

private:
  void testPoses()
  {
    if (test_index_ >= test_poses_.size()) {
      RCLCPP_INFO(get_logger(), "All tests complete!");
      timer_->cancel();
      return;
    }
    
    auto & test = test_poses_[test_index_];
    RCLCPP_INFO(get_logger(), "\n=== Test %zu/%zu: %s ===", 
                test_index_ + 1, test_poses_.size(), test.name.c_str());
    RCLCPP_INFO(get_logger(), "Pose: [%.2f, %.2f, %.2f]", 
                test.x, test.y, test.z);
    
    // Create pose
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = test.x;
    pose.pose.position.y = test.y;
    pose.pose.position.z = test.z;
    pose.pose.orientation.w = 1.0;  // No rotation
    
    // Select arm based on Y
    auto group = (test.y < -0.2) ? left_group_ : right_group_;
    std::string arm_name = (test.y < -0.2) ? "LEFT" : "RIGHT";
    
    // Try to plan
    group->setPoseTarget(pose);
    group->setPlanningTime(3.0);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = group->plan(plan);
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(get_logger(), "✓ %s arm can reach this pose!", arm_name.c_str());
      RCLCPP_INFO(get_logger(), "Executing...");
      group->execute(plan);
      rclcpp::sleep_for(std::chrono::seconds(2));
    } else {
      RCLCPP_WARN(get_logger(), "✗ %s arm CANNOT reach this pose", arm_name.c_str());
    }
    
    test_index_++;
  }
  
  struct TestPose {
    std::string name;
    double x, y, z;
  };
  
  std::vector<TestPose> test_poses_ = {
    {"Forward center, mid height", 0.4, 0.0, 0.5},
    {"Forward center, high", 0.4, 0.0, 0.7},
    {"Forward left, mid height", 0.4, -0.3, 0.5},
    {"Forward left, table height", 0.4, -0.3, 0.3},
    {"Forward right, mid height", 0.4, 0.3, 0.5},
    {"Side left, mid height", 0.2, -0.4, 0.5},
    {"Side right, mid height", 0.2, 0.4, 0.5},
    {"Close forward, high", 0.25, 0.0, 0.6},
  };
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t test_index_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmWorkspaceTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
