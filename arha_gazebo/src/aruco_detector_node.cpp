#include "arha_gazebo/aruco_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/calib3d.hpp>

namespace arha_gazebo
{

ArucoDetectorNode::ArucoDetectorNode()
: Node("aruco_detector"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Declare parameters
  declare_parameter<double>("marker_size", 0.035);
  declare_parameter<std::string>("camera_topic", "/camera_chest/color/image");
  declare_parameter<std::string>("camera_info_topic", "/camera_chest/color/camera_info");
  declare_parameter<std::string>("depth_topic", "/camera_chest/depth/points");
  declare_parameter<std::string>("target_frame", "base_link");
  declare_parameter<bool>("publish_debug_images", false);

  // Get parameters
  marker_size_ = get_parameter("marker_size").as_double();
  camera_topic_ = get_parameter("camera_topic").as_string();
  target_frame_ = get_parameter("target_frame").as_string();
  publish_debug_images_ = get_parameter("publish_debug_images").as_bool();

  // Initialize ArUco detection
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  detector_params_ = cv::aruco::DetectorParameters::create();

  // Create subscriptions
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    get_parameter("camera_info_topic").as_string(), 10,
    std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    camera_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    get_parameter("depth_topic").as_string(), rclcpp::SensorDataQoS(),
    std::bind(&ArucoDetectorNode::pointCloudCallback, this, std::placeholders::_1));

  // Create publishers
  marker_poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
    "detected_markers", 10);

  filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "filtered_point_cloud", 10);

  RCLCPP_INFO(get_logger(), "ArUco Detector Node initialized");
  RCLCPP_INFO(get_logger(), "  Camera topic: %s", camera_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  Marker size: %.3f m", marker_size_);
  RCLCPP_INFO(get_logger(), "  Target frame: %s", target_frame_.c_str());
}

// ============================================================================
// Callbacks
// ============================================================================

void ArucoDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_.ready) {
    camera_.camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
    camera_.dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F,
        const_cast<double *>(msg->d.data())).clone();
    camera_.optical_frame = msg->header.frame_id;
    camera_.ready = true;
    RCLCPP_INFO(get_logger(), "Camera calibrated: %s", camera_.optical_frame.c_str());
  }
}

void ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!camera_.ready) {
    return;
  }

  detectMarkers(msg);
}

void ArucoDetectorNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // For now, just republish the point cloud
  // In future, we can filter it based on detected markers
  filtered_cloud_pub_->publish(*msg);
}

// ============================================================================
// Helper Methods
// ============================================================================

void ArucoDetectorNode::detectMarkers(const sensor_msgs::msg::Image::SharedPtr & msg)
{
  // Convert ROS image to OpenCV format
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
    // Publish empty pose array if no markers detected
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = msg->header.stamp;
    pose_array.header.frame_id = target_frame_;
    marker_poses_pub_->publish(pose_array);
    return;
  }

  RCLCPP_INFO(get_logger(), "Detected %zu ArUco marker(s)", ids.size());

  // Estimate 3D poses
  cv::Mat rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
    corners, marker_size_, camera_.camera_matrix, camera_.dist_coeffs, rvecs, tvecs);

  // Get transform from camera to target frame
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      target_frame_, camera_.optical_frame, msg->header.stamp,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  // Process detected markers
  std::vector<DetectedMarker> detected_markers;
  for (size_t i = 0; i < ids.size(); ++i) {
    // Create pose in camera frame
    geometry_msgs::msg::PoseStamped pose_cam;
    pose_cam.header.frame_id = camera_.optical_frame;
    pose_cam.header.stamp = msg->header.stamp;
    pose_cam.pose.position.x = tvecs.at<cv::Vec3d>(i, 0)[0];
    pose_cam.pose.position.y = tvecs.at<cv::Vec3d>(i, 0)[1];
    pose_cam.pose.position.z = tvecs.at<cv::Vec3d>(i, 0)[2];

    // Convert rotation vector to quaternion
    cv::Mat rot;
    cv::Rodrigues(rvecs.row(i), rot);
    tf2::Matrix3x3 tf_rot(
      rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
      rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
      rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
    tf2::Quaternion q;
    tf_rot.getRotation(q);
    pose_cam.pose.orientation = tf2::toMsg(q);

    // Transform to target frame
    geometry_msgs::msg::PoseStamped pose_target;
    tf2::doTransform(pose_cam, pose_target, transform);

    // Store detected marker
    DetectedMarker marker;
    marker.id = ids[i];
    marker.pose = pose_target;
    marker.confidence = 1.0;  // Can be improved with corner detection quality
    detected_markers.push_back(marker);

    RCLCPP_INFO(get_logger(), "Marker ID %d at [%.3f, %.3f, %.3f]",
      marker.id, pose_target.pose.position.x,
      pose_target.pose.position.y, pose_target.pose.position.z);
  }

  publishMarkers(detected_markers);
}

void ArucoDetectorNode::publishMarkers(const std::vector<DetectedMarker> & markers)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = this->now();
  pose_array.header.frame_id = target_frame_;

  for (const auto & marker : markers) {
    pose_array.poses.push_back(marker.pose.pose);
  }

  marker_poses_pub_->publish(pose_array);
}

}  // namespace arha_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arha_gazebo::ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
