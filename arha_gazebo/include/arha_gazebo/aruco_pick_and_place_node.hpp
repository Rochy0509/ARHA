#ifndef ARHA_GAZEBO__ARUCO_PICK_AND_PLACE_NODE_HPP_
#define ARHA_GAZEBO__ARUCO_PICK_AND_PLACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/aruco.hpp>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

namespace arha_gazebo
{

// Motion planning constants
constexpr double VELOCITY_SCALING = 0.3;
constexpr double ACCELERATION_SCALING = 0.3;
constexpr double GRIPPER_OPEN_POSITION = 0.025;
constexpr double GRIPPER_CLOSE_POSITION = 0.005;
constexpr int GRIPPER_MOTION_DURATION_SEC = 1;

// Marker positioning constants
constexpr double MARKER_THICKNESS_COMPENSATION = 0.015;  
constexpr double PREGRASP_OFFSET_Z = 0.12;  
constexpr double GRASP_OFFSET_Z = 0.03;  
constexpr double RETREAT_OFFSET_Z = 0.05;  
constexpr double DROP_PREGRASP_OFFSET_Z = 0.1;  

// Planning parameters
constexpr double CARTESIAN_PATH_STEP_SIZE = 0.01;  
constexpr double CARTESIAN_PATH_MIN_FRACTION = 0.9;  
constexpr double TF_LOOKUP_TIMEOUT = 0.1;  

// Workspace division boundary (Y coordinate in base_link)
constexpr double LEFT_RIGHT_ARM_BOUNDARY_Y = -0.2;

struct CameraModel
{
  bool ready{false};
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::string optical_frame;
};

struct TargetCube
{
  int id{0};
  geometry_msgs::msg::PoseStamped pose;
  rclcpp::Time last_seen;
};

class ArucoPickAndPlaceNode : public rclcpp::Node
{
public:
  ArucoPickAndPlaceNode();

private:
  // Camera callbacks
  void chestInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void chestImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  // ArUco detection
  void detectMarkersChest(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    const CameraModel & model,
    const std::string & optical_frame);
  
  // Collision scene setup
  void addTableCollisionObject();
  
  // MoveIt initialization
  void tryInitializeMoveGroups();
  
  // Target queue processing
  void processQueue();
  
  // Pick and place execution
  void executePickAndPlace(const TargetCube & target);
  
  // Motion planning utilities
  bool planAndExecute(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group,
    const geometry_msgs::msg::PoseStamped & pose);
  
  bool planCartesian(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group,
    const std::vector<geometry_msgs::msg::PoseStamped> & waypoints);
  
  // Gripper control
  void commandGripper(
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub,
    bool close);
  
  // Helper methods
  void orientDownward(geometry_msgs::msg::Pose & pose);
  void updateCameraModel(
    const sensor_msgs::msg::CameraInfo::SharedPtr & msg,
    CameraModel& model);
  
  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  // Camera subscriptions (chest only now)
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr chest_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr chest_image_sub_;
  
  // Gripper publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_gripper_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_gripper_pub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr move_group_init_timer_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  
  // Camera model (chest only)
  CameraModel chest_camera_;
  
  // MoveIt interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> scene_interface_;
  bool move_groups_initialized_{false};
  
  // Drop zone
  geometry_msgs::msg::PoseStamped drop_pose_;
  
  // ArUco detection
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  
  // Target queue
  std::deque<TargetCube> target_queue_;
  std::mutex queue_mutex_;
};

}  // namespace arha_gazebo

#endif  // ARHA_GAZEBO__ARUCO_PICK_AND_PLACE_NODE_HPP_
