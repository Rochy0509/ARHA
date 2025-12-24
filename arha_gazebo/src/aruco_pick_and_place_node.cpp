#include "arha_gazebo/aruco_pick_and_place_node.hpp"
#include <builtin_interfaces/msg/duration.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <chrono>
#include <cmath>

namespace arha_gazebo
{

ArucoPickAndPlaceNode::ArucoPickAndPlaceNode()
: Node("aruco_pick_and_place_cpp"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Declare parameters
  declare_parameter<double>("marker_size", 0.035);
  declare_parameter<double>("drop_x", -0.18);
  declare_parameter<double>("drop_y", -0.45);
  declare_parameter<double>("drop_z", 0.42);
  declare_parameter<std::string>("robot_description", "");
  declare_parameter<std::string>("robot_description_semantic", "");

  // Initialize ArUco detection
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  detector_params_ = cv::aruco::DetectorParameters::create();

  // Initialize planning scene
  scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  // Gripper control publishers
  left_gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/left_gripper_controller/joint_trajectory", 10);
  right_gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/right_gripper_controller/joint_trajectory", 10);

  // Camera subscriptions (chest only)
  using std::placeholders::_1;
  chest_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_chest/camera_chest/camera_info", 10,
    std::bind(&ArucoPickAndPlaceNode::chestInfoCallback, this, _1));
  
  chest_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera_chest/camera_chest/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArucoPickAndPlaceNode::chestImageCallback, this, _1));

  // Configure drop zone
  drop_pose_.header.frame_id = "base_link";
  drop_pose_.pose.position.x = get_parameter("drop_x").as_double();
  drop_pose_.pose.position.y = get_parameter("drop_y").as_double();
  drop_pose_.pose.position.z = get_parameter("drop_z").as_double();
  drop_pose_.pose.orientation.w = 1.0;

  // Start timers
  move_group_init_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ArucoPickAndPlaceNode::tryInitializeMoveGroups, this));

  process_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ArucoPickAndPlaceNode::processQueue, this));

  RCLCPP_INFO(get_logger(), "ArUco Pick and Place Node initialized");
  RCLCPP_INFO(get_logger(), "Direct chest detection -> pick-and-place workflow");
}

void ArucoPickAndPlaceNode::chestInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  updateCameraModel(msg, chest_camera_);
}

void ArucoPickAndPlaceNode::chestImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  detectMarkersChest(msg, chest_camera_, chest_camera_.optical_frame);
}

void ArucoPickAndPlaceNode::updateCameraModel(
  const sensor_msgs::msg::CameraInfo::SharedPtr & msg,
  CameraModel & model)
{
  if (!model.ready) {
    model.camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
    model.dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F,
        const_cast<double *>(msg->d.data())).clone();
    model.optical_frame = msg->header.frame_id;
    model.ready = true;
    RCLCPP_INFO(get_logger(), "Camera calibrated: %s", model.optical_frame.c_str());
  }
}

void ArucoPickAndPlaceNode::detectMarkersChest(
  const sensor_msgs::msg::Image::SharedPtr & msg,
  const CameraModel & model,
  const std::string & optical_frame)
{
  if (!model.ready || !move_groups_initialized_) {
    return;
  }

  // Convert ROS image to OpenCV
  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", ex.what());
    return;
  }

  // Detect ArUco markers
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_params_);

  if (ids.empty()) {
    return;
  }

  RCLCPP_INFO(get_logger(), "Chest camera detected %zu ArUco marker(s)", ids.size());

  // Estimate marker poses
  std::vector<cv::Vec3d> rvecs, tvecs;
  double marker_size = get_parameter("marker_size").as_double();
  cv::aruco::estimatePoseSingleMarkers(corners, marker_size, model.camera_matrix,
                                        model.dist_coeffs, rvecs, tvecs);

  // Transform to base_link and add to queue
  for (size_t i = 0; i < ids.size(); ++i) {
    // Check if marker already in queue
    bool already_in_queue = false;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      for (const auto & target : target_queue_) {
        if (target.id == ids[i]) {
          already_in_queue = true;
          break;
        }
      }
    }
    
    if (already_in_queue) {
      continue;
    }

    // Transform marker pose to base_link
    geometry_msgs::msg::PoseStamped pose_camera, pose_base;
    pose_camera.header.frame_id = optical_frame;
    pose_camera.header.stamp = msg->header.stamp;
    
    // Position from ArUco estimate
    pose_camera.pose.position.x = tvecs[i][0];
    pose_camera.pose.position.y = tvecs[i][1];
    pose_camera.pose.position.z = tvecs[i][2];
    
    // Orientation from rotation vector
    cv::Mat rot_mat;
    cv::Rodrigues(rvecs[i], rot_mat);
    tf2::Matrix3x3 tf_rot(
      rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
      rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
      rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
    
    tf2::Quaternion tf_quat;
    tf_rot.getRotation(tf_quat);
    pose_camera.pose.orientation.x = tf_quat.x();
    pose_camera.pose.orientation.y = tf_quat.y();
    pose_camera.pose.orientation.z = tf_quat.z();
    pose_camera.pose.orientation.w = tf_quat.w();

    // Transform to base_link
    try {
      tf_buffer_.transform(pose_camera, pose_base, "base_link",
                           tf2::durationFromSec(TF_LOOKUP_TIMEOUT));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
      continue;
    }

    // Compensate for marker thickness
    pose_base.pose.position.z -= MARKER_THICKNESS_COMPENSATION;

    // Add to queue
    TargetCube target;
    target.id = ids[i];
    target.pose = pose_base;
    target.last_seen = msg->header.stamp;
    
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      target_queue_.push_back(target);
      
      RCLCPP_INFO(get_logger(),
        "Added marker ID %d to queue at position [%.3f, %.3f, %.3f] in base_link. Queue size: %zu",
        target.id, pose_base.pose.position.x, pose_base.pose.position.y, pose_base.pose.position.z,
        target_queue_.size());
    }
  }
}

void ArucoPickAndPlaceNode::addTableCollisionObject()
{
  if (!scene_interface_) {
    return;
  }

  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "base_link";
  table.id = "table";

  // Define table as a box
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 1.5;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 2.0;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.05;

  // Position table below robot base
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.3;
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.025;
  table_pose.orientation.w = 1.0;

  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  scene_interface_->applyCollisionObject(table);
  RCLCPP_INFO(get_logger(), "Added table collision object to planning scene");
}

void ArucoPickAndPlaceNode::tryInitializeMoveGroups()
{
  if (move_groups_initialized_) {
    move_group_init_timer_->cancel();
    return;
  }

  try {
    left_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "left_arm");
    right_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "right_arm");

    // Configure planning
    left_group_->setMaxVelocityScalingFactor(VELOCITY_SCALING);
    left_group_->setMaxAccelerationScalingFactor(ACCELERATION_SCALING);
    right_group_->setMaxVelocityScalingFactor(VELOCITY_SCALING);
    right_group_->setMaxAccelerationScalingFactor(ACCELERATION_SCALING);

    move_groups_initialized_ = true;
    move_group_init_timer_->cancel();

    RCLCPP_INFO(get_logger(), "MoveIt move groups initialized successfully");

    // Add table collision
    addTableCollisionObject();

  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "MoveIt initialization failed: %s", e.what());
  }
}

void ArucoPickAndPlaceNode::processQueue()
{
  if (!move_groups_initialized_ || target_queue_.empty()) {
    return;
  }

  std::unique_lock<std::mutex> lock(queue_mutex_);
  TargetCube target = target_queue_.front();
  target_queue_.pop_front();
  lock.unlock();

  RCLCPP_INFO(get_logger(), "Processing marker ID %d - executing pick-and-place", target.id);

  executePickAndPlace(target);
}

void ArucoPickAndPlaceNode::executePickAndPlace(const TargetCube & target)
{
  // Select arm based on target Y position
  auto & move_group = (target.pose.pose.position.y < LEFT_RIGHT_ARM_BOUNDARY_Y) ?
    left_group_ : right_group_;
  auto & gripper_pub = (move_group == left_group_) ?
    left_gripper_pub_ : right_gripper_pub_;
  const std::string arm_name = (move_group == left_group_) ? "LEFT" : "RIGHT";

  RCLCPP_INFO(get_logger(), "Using %s arm for marker ID %d", arm_name.c_str(), target.id);

  // Create poses for pick sequence
  geometry_msgs::msg::PoseStamped pregrasp, grasp, retreat;
  pregrasp.header.frame_id = "base_link";
  grasp.header.frame_id = "base_link";
  retreat.header.frame_id = "base_link";

  // Pregrasp: approach from front (arms can't reach close to base)
  pregrasp.pose = target.pose.pose;
  pregrasp.pose.position.x = std::max(0.25, target.pose.pose.position.x);  // At least 25cm forward
  pregrasp.pose.position.z += PREGRASP_OFFSET_Z;
  orientDownward(pregrasp.pose);

  // Grasp: at target
  grasp.pose = target.pose.pose;
  grasp.pose.position.z += GRASP_OFFSET_Z;
  orientDownward(grasp.pose);

  // Retreat: above pregrasp
  retreat.pose = pregrasp.pose;
  retreat.pose.position.z += RETREAT_OFFSET_Z;

  // Drop zone poses
  geometry_msgs::msg::PoseStamped drop_pre = drop_pose_;
  drop_pre.pose.position.z += DROP_PREGRASP_OFFSET_Z;
  orientDownward(drop_pre.pose);

  geometry_msgs::msg::PoseStamped drop = drop_pose_;
  orientDownward(drop.pose);

  // Execute pick-and-place sequence
  RCLCPP_INFO(get_logger(), "Stage 1: Opening gripper");
  commandGripper(gripper_pub, false);
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(get_logger(), "Stage 2: Moving to pregrasp pose");
  if (!planAndExecute(move_group, pregrasp)) {
    RCLCPP_ERROR(get_logger(), "Failed to reach pregrasp - aborting");
    return;
  }

  RCLCPP_INFO(get_logger(), "Stage 3: Lowering to grasp pose");
  if (!planCartesian(move_group, {grasp})) {
    RCLCPP_ERROR(get_logger(), "Failed to lower to grasp - aborting");
    return;
  }

  RCLCPP_INFO(get_logger(), "Stage 4: Closing gripper");
  commandGripper(gripper_pub, true);
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(get_logger(), "Stage 5: Retreating with object");
  planCartesian(move_group, {retreat});

  RCLCPP_INFO(get_logger(), "Stage 6: Moving to drop zone");
  planAndExecute(move_group, drop_pre);
  planCartesian(move_group, {drop});

  RCLCPP_INFO(get_logger(), "Stage 7: Releasing object");
  commandGripper(gripper_pub, false);
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(get_logger(), "Stage 8: Retreating from drop zone");
  planCartesian(move_group, {drop_pre});

  RCLCPP_INFO(get_logger(), "Pick-and-place completed for marker ID %d", target.id);
}

bool ArucoPickAndPlaceNode::planAndExecute(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group,
  const geometry_msgs::msg::PoseStamped & pose)
{
  group->setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = group->plan(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }

  return group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool ArucoPickAndPlaceNode::planCartesian(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group,
  const std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
{
  // Convert PoseStamped to Pose
  std::vector<geometry_msgs::msg::Pose> poses;
  for (const auto & ps : waypoints) {
    poses.push_back(ps.pose);
  }

  // Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = group->computeCartesianPath(
    poses, CARTESIAN_PATH_STEP_SIZE, 0.0, trajectory);

  if (fraction < CARTESIAN_PATH_MIN_FRACTION) {
    RCLCPP_WARN(get_logger(), "Cartesian path incomplete (%.2f%%)", fraction * 100.0);
    return false;
  }

  // Execute trajectory
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  return group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

void ArucoPickAndPlaceNode::commandGripper(
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub,
  bool close)
{
  trajectory_msgs::msg::JointTrajectory jt;
  const bool left = (pub == left_gripper_pub_);
  const std::string prefix = left ? "left" : "right";

  jt.joint_names = {
    prefix + "_robotiq_hande_left_finger_joint",
    prefix + "_robotiq_hande_right_finger_joint"
  };

  trajectory_msgs::msg::JointTrajectoryPoint pt;
  const double position = close ? GRIPPER_CLOSE_POSITION : GRIPPER_OPEN_POSITION;
  pt.positions = {position, position};

  builtin_interfaces::msg::Duration duration_msg;
  duration_msg.sec = GRIPPER_MOTION_DURATION_SEC;
  duration_msg.nanosec = 0;
  pt.time_from_start = duration_msg;

  jt.points.push_back(pt);
  pub->publish(jt);
}

void ArucoPickAndPlaceNode::orientDownward(geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);
  pose.orientation = tf2::toMsg(q);
}

}  // namespace arha_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arha_gazebo::ArucoPickAndPlaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
